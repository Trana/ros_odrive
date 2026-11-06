#include "support_fallback.hpp"
#include "paced_commands.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>

using odrive_ros2_control::SupportFallback;
using namespace std::chrono_literals;

namespace {
void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

SupportFallback::Policy policy() {
    return {100ms, 40ms, 50ms, 50ms, 0.1};
}

std::vector<SupportFallback::AxisObservation> observations(
    SupportFallback::Clock::time_point now, size_t axes = 12
) {
    std::vector<SupportFallback::AxisObservation> result(axes);
    for (size_t axis = 0; axis < axes; ++axis) {
        result[axis] = {
            true,
            true,
            true,
            static_cast<double>(axis) / 10.0,
            static_cast<double>(axis) / 10.0,
            now - 1ms,
            now - 1ms,
            now - 1ms,
        };
    }
    return result;
}

SupportFallback::FrozenBatch batch(SupportFallback::Clock::time_point now, size_t next = 4) {
    SupportFallback::FrozenBatch result;
    result.activation_epoch = 7;
    result.generation = 11;
    result.snapshot_id = 13;
    result.position_hold_allowed = true;
    result.source_time = now - 10ms;
    result.batch_started = now - 20ms;
    result.next_axis = next;
    result.position_only_frames.resize(12);
    result.target_rad.resize(12);
    for (size_t axis = 0; axis < 12; ++axis) {
        can_frame frame{};
        frame.can_id = static_cast<canid_t>(axis);
        frame.data[0] = 42;
        result.position_only_frames[axis] = frame;
        result.target_rad[axis] = static_cast<double>(axis) / 10.0;
    }
    return result;
}
}

int main() {
    try {
        const auto now = SupportFallback::Clock::time_point{} + 1s;
        // Fault evidence must identify the gate without changing its decision.
        const auto rejected = [&](auto mutate, const char* expected, size_t axis) {
            SupportFallback f;
            f.configure(policy(), 12);
            f.activate(7);
            auto b = batch(now);
            auto o = observations(now);
            mutate(b, o);
            check(f.on_batch_deadline(b, o, now, false) == SupportFallback::Decision::request_idle,
                  "diagnostics must retain rejection");
            check(std::string_view(f.failure().check) == expected && f.failure().axis == axis,
                  "admission must identify failed check and axis");
            check(f.failure().generation == 11 && f.failure().snapshot_id == 13,
                  "failure must preserve frozen identity");
            const auto first = f.failure();
            f.submission_failed();
            check(f.failure().check == first.check && f.failure().actual == first.actual,
                  "shutdown must preserve first cause");
            return first;
        };
        const auto tracking = rejected([](auto&, auto& o) { o[4].position_rad += .3; },
                                       "tracking_error_rad", 4);
        check(std::abs(tracking.actual - .3) < 1e-9 && tracking.limit == .1
                  && tracking.target_rad == .4 && std::abs(tracking.position_rad - .7) < 1e-9,
              "tracking diagnostic must contain measured values and limit");
        rejected([&](auto&, auto& o) { o[2].heartbeat_receipt = now - 60ms; }, "heartbeat_age_ms", 2);
        rejected([&](auto&, auto& o) { o[3].error_receipt = now - 60ms; }, "error_age_ms", 3);
        rejected([&](auto&, auto& o) { o[5].encoder_receipt = now - 60ms; }, "encoder_age_ms", 5);
        rejected([](auto&, auto& o) { o[6].closed_loop = false; }, "closed_loop", 6);
        rejected([](auto&, auto& o) { o[7].error_free = false; }, "motor_error", 7);
        rejected([&](auto& b, auto&) { b.source_time = now - 101ms; }, "snapshot_age_ms", size_t(-1));
        rejected([&](auto& b, auto&) { b.batch_started = now - 41ms; }, "completion_span_ms", size_t(-1));
        SupportFallback lost;
        lost.configure(policy(), 12); lost.activate(7);
        check(lost.on_batch_deadline(batch(now, 12), observations(now), now, false)
                  == SupportFallback::Decision::recover_hold, "healthy admission");
        auto changed = observations(now);
        changed[8].position_rad += .4;
        check(!lost.next_frame(changed, now + 1ms, false), "tracking loss still stops hold");
        check(std::string_view(lost.failure().phase) == "holding"
                  && std::string_view(lost.failure().check) == "tracking_error_rad"
                  && lost.failure().axis == 8, "ongoing hold failure must identify phase and axis");
        SupportFallback starved;
        starved.configure(policy(), 12); starved.activate(7);
        starved.on_batch_deadline(batch(now, 12), observations(now), now, false);
        check(!starved.next_frame(observations(now + 51ms), now + 51ms, false), "submission expiry");
        check(std::string_view(starved.failure().check) == "submission_age_ms"
                  && starved.failure().actual == 51, "submission expiry evidence");
        SupportFallback streaming;
        streaming.configure(policy(), 12);
        streaming.activate(7);
        for (size_t axis = 0; axis < 11; ++axis)
            streaming.streaming_submitted(axis, 7, 5, 9,
                                           static_cast<double>(axis) / 10.0, now);
        auto streaming_report = streaming.report(observations(now), now + 1ms);
        check(streaming_report.generation == 0 && !streaming_report.hold_confirmed,
              "a partial normal batch must not replace coherent evidence");
        streaming.streaming_submitted(11, 7, 5, 9, 1.1, now);
        streaming_report = streaming.report(observations(now), now + 1ms);
        check(streaming_report.generation == 5 && streaming_report.snapshot_id == 9
                  && streaming_report.hold_confirmed,
              "a complete normal batch must publish observed coherent evidence");
        streaming.streaming_submitted(0, 7, 6, 10, 0.0, now + 2ms);
        streaming_report = streaming.report(observations(now), now + 2ms);
        check(streaming_report.generation == 5 && streaming_report.hold_confirmed,
              "a pending replacement must not make complete evidence flicker");

        SupportFallback unreviewed_stream;
        unreviewed_stream.configure({}, 12);
        unreviewed_stream.activate(7);
        for (size_t axis = 0; axis < 12; ++axis)
            unreviewed_stream.streaming_submitted(axis, 7, 5, 9,
                static_cast<double>(axis) / 10.0, now);
        const auto unreviewed_report = unreviewed_stream.report(observations(now), now + 1ms);
        check(!unreviewed_report.policy_ready && !unreviewed_report.hold_confirmed
                  && unreviewed_report.axes[0].level == SupportFallback::Level::submitted,
              "normal submissions remain visible without an approved observation policy");

        SupportFallback fallback;
        fallback.configure(policy(), 12);
        fallback.activate(7);
        auto feedback = observations(now);

        check(
            fallback.on_batch_deadline(batch(now), feedback, now, false)
                == SupportFallback::Decision::recover_hold,
            "current authorized batch must enter fallback"
        );
        check(fallback.state() == SupportFallback::State::finishing_batch,
              "partial batch must finish before repeat hold");
        for (size_t axis = 4; axis < 12; ++axis) {
            auto next = fallback.next_frame(feedback, now + (axis - 4) * 100us, false);
            check(next && next->first == axis && next->second.data[0] == 42,
                  "fallback must finish the same frozen generation in axis order");
            fallback.submitted(axis, now + (axis - 4) * 100us);
        }
        check(fallback.state() == SupportFallback::State::holding,
              "completed batch must enter repeated position hold");
        for (size_t axis = 0; axis < 4; ++axis) {
            auto next = fallback.next_frame(feedback, now + 1ms + axis * 100us, false);
            check(next && next->first == axis, "hold must repeat the frozen batch");
            fallback.submitted(axis, now + 1ms + axis * 100us);
        }
        auto report = fallback.report(feedback, now + 2ms);
        check(report.generation == 11 && report.snapshot_id == 13,
              "report must preserve generation identity");
        check(report.hold_confirmed, "fresh tracking evidence must confirm all axes");
        check(!report.balance_verified, "joint hold must not claim balance");

        feedback[3].closed_loop = false;
        report = fallback.report(feedback, now + 2ms);
        check(!report.hold_confirmed, "one unsupported axis must clear confirmation");
        check(report.axes[3].level == SupportFallback::Level::submitted,
              "socket submission without motor evidence remains submitted");

        SupportFallback stopped;
        stopped.configure(policy(), 12);
        stopped.activate(7);
        check(stopped.on_batch_deadline(batch(now), observations(now), now, true)
                  == SupportFallback::Decision::request_idle,
              "Stop before admission must request IDLE");
        check(stopped.state() == SupportFallback::State::stopping,
              "Stop must remain distinct from unavailable support");

        SupportFallback stop_race;
        stop_race.configure(policy(), 12);
        stop_race.activate(7);
        check(stop_race.on_batch_deadline(batch(now), observations(now), now, false)
                  == SupportFallback::Decision::recover_hold,
              "race setup");
        check(!stop_race.next_frame(observations(now), now + 1ms, true),
              "Stop during partial completion must suppress the next frame");
        check(stop_race.requires_idle(), "Stop race must select IDLE");

        for (const auto mutate : {0, 1, 2, 3, 4, 5, 6, 7}) {
            SupportFallback rejected;
            rejected.configure(policy(), 12);
            rejected.activate(7);
            auto candidate = batch(now);
            auto state = observations(now);
            if (mutate == 0) candidate.activation_epoch = 8;
            if (mutate == 1) candidate.source_time = now - 101ms;
            if (mutate == 2) state[0].encoder_receipt = now - 51ms;
            if (mutate == 3) state[0].error_free = false;
            if (mutate == 4) state[0].position_mode = false;
            if (mutate == 5) state[0].position_rad = NAN;
            if (mutate == 6) state[0].position_rad += 0.2;
            if (mutate == 7) candidate.position_only_frames[0].reset();
            check(rejected.on_batch_deadline(candidate, state, now, false)
                      == SupportFallback::Decision::request_idle,
                  "stale, mismatched, non-position, or faulted evidence must reject hold");
        }

        SupportFallback tracking_lost;
        tracking_lost.configure(policy(), 12);
        tracking_lost.activate(7);
        check(tracking_lost.on_batch_deadline(batch(now), observations(now), now, false)
                  == SupportFallback::Decision::recover_hold,
              "tracking-loss setup");
        auto diverged = observations(now + 1ms);
        diverged[0].position_rad += 0.2;
        check(!tracking_lost.next_frame(diverged, now + 1ms, false),
              "tracking outside the reviewed bound must stop support");
        check(tracking_lost.requires_idle(), "lost tracking must request IDLE");

        SupportFallback no_policy;
        no_policy.configure({}, 12);
        no_policy.activate(7);
        check(no_policy.on_batch_deadline(batch(now), observations(now), now, false)
                  == SupportFallback::Decision::request_idle,
              "missing reviewed policy must keep fail-closed IDLE");

        SupportFallback sustained;
        sustained.configure(policy(), 12);
        sustained.activate(7);
        check(sustained.on_batch_deadline(batch(now), observations(now), now, false)
                  == SupportFallback::Decision::recover_hold,
              "sustained hold setup");
        // Advance a virtual minute with fresh motor evidence and timely per-axis
        // submissions. Old duration expiry must not remove healthy support.
        for (int tick = 0; tick < 60000; ++tick) {
            const auto at = now + tick * 1ms;
            const auto next = sustained.next_frame(observations(at), at, false);
            check(next.has_value(), "healthy sustained hold must not expire");
            sustained.submitted(next->first, at);
        }
        check(sustained.report(observations(now + 60s), now + 60s).hold_confirmed,
              "sustained hold must retain all-axis confirmation");
        check(!sustained.next_frame(observations(now + 60s), now + 60s, true),
              "explicit Stop must terminate sustained hold");
        check(sustained.state() == SupportFallback::State::stopping,
              "explicit Stop must retain priority");

        for (int cause = 0; cause < 6; ++cause) {
            SupportFallback lost;
            lost.configure(policy(), 12);
            lost.activate(7);
            check(lost.on_batch_deadline(batch(now), observations(now), now, false)
                      == SupportFallback::Decision::recover_hold, "loss setup");
            for (int tick = 0; tick < 1000; ++tick) {
                auto at = now + tick * 1ms;
                auto next = lost.next_frame(observations(at), at, false);
                check(next.has_value(), "healthy setup must continue");
                lost.submitted(next->first, at);
            }
            auto at = now + 1000ms;
            auto unhealthy = observations(at);
            if (cause == 0) unhealthy[0].encoder_receipt = at - 51ms;
            if (cause == 1) unhealthy[0].closed_loop = false;
            if (cause == 2) unhealthy[0].error_free = false;
            if (cause == 3) unhealthy[0].position_rad += .2;
            if (cause == 4) lost.submission_failed();
            if (cause == 5) { at += 51ms; unhealthy = observations(at); }
            check(!lost.next_frame(unhealthy, at, false),
                  "lost feedback, motor health, tracking or submissions must stop hold");
            check(lost.requires_idle(), "actual hold failure retains IDLE priority");
        }

        odrive_ros2_control::PacedCommands paced;
        SupportFallback late_final;
        late_final.configure(policy(), 12);
        late_final.activate(7);
        check(late_final.on_batch_deadline(batch(now, 11), observations(now), now, false)
                  == SupportFallback::Decision::recover_hold, "late final setup");
        auto final_frame = late_final.next_frame(observations(now), now, false);
        check(final_frame && final_frame->first == 11, "final axis selected within bound");
        late_final.submitted(11, now + 21ms);  // Batch already consumed 20 of 40 ms.
        check(late_final.requires_idle(),
              "late final send return cannot enter continued hold past completion bound");

        paced.configure(12, 50, 250, false, false, true);
        auto paced_now = odrive_ros2_control::PacedCommands::Clock::time_point{} + 2s;
        paced.activate(paced_now);
        for (size_t axis = 0; axis < 12; ++axis) {
            can_frame motion{};
            motion.can_id = static_cast<canid_t>(axis);
            motion.data[0] = 99;
            can_frame hold = motion;
            hold.data[1] = 1;
            paced.set(axis, motion, hold, static_cast<double>(axis) / 10.0);
        }
        paced.commit(paced_now, 7, 11, 13, true);
        for (size_t axis = 0; axis < 4; ++axis) {
            check(paced.due(paced_now).has_value(), "partial batch setup");
            paced.finish_submission(paced_now, true);
        }
        for (uint64_t generation = 12; generation <= 15; ++generation)
            paced.commit(paced_now, 7, generation, generation + 2, true);
        paced_now += 20ms;
        check(!paced.due(paced_now) && paced.faulted(),
              "late partial batch must expose a deadline fault");
        auto frozen = paced.take_recovery_batch();
        check(frozen && frozen->generation == 11 && frozen->next_axis == 4,
              "pacer must preserve frozen generation and cursor for fallback");
        check(frozen->position_only_frames[4]->data[1] == 1,
              "fallback copy must use the sanitized position frame");
        SupportFallback composed;
        composed.configure(policy(), 12);
        composed.activate(7);
        check(composed.on_batch_deadline(*frozen, observations(paced_now), paced_now, false)
                  == SupportFallback::Decision::recover_hold,
              "the current authorized generation must admit its frozen batch");

        SupportFallback stale_activation;
        stale_activation.configure(policy(), 12);
        stale_activation.activate(8);
        check(stale_activation.on_batch_deadline(
                  *frozen, observations(paced_now), paced_now, false)
                  == SupportFallback::Decision::request_idle,
              "a new hardware activation must reject an older frozen batch");

        std::cout << "Generation-fenced support fallback checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

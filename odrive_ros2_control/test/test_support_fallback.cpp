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
    return {100ms, 40ms, 50ms, 50ms, 500ms, 0.1};
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

        SupportFallback expired;
        expired.configure(policy(), 12);
        expired.activate(7);
        check(expired.on_batch_deadline(batch(now), observations(now), now, false)
                  == SupportFallback::Decision::recover_hold,
              "expiry setup");
        check(!expired.next_frame(observations(now), now + 501ms, false),
              "hold duration must be bounded");
        check(expired.requires_idle(), "expired hold must request IDLE");

        odrive_ros2_control::PacedCommands paced;
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

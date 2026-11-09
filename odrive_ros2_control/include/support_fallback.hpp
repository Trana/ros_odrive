#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <linux/can.h>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace odrive_ros2_control {

class SupportFallback {
public:
    using Clock = std::chrono::steady_clock;
    using Duration = Clock::duration;

    enum class State { disabled, streaming, finishing_batch, holding, unavailable, stopping };
    enum class Level { requested, submitted, observed, unavailable };

    // Fixed-size first-cause evidence. No strings or allocation on the sender path.
    struct Failure {
        const char* phase = "none";
        const char* check = "none";
        size_t axis = static_cast<size_t>(-1);
        double actual = NAN;
        double limit = NAN;
        double target_rad = NAN;
        double position_rad = NAN;
        double snapshot_age_ms = NAN;
        double batch_age_ms = NAN;
        uint64_t generation = 0;
        uint64_t snapshot_id = 0;
        uint64_t activation_epoch = 0;
    };
    const Failure& failure() const { return failure_; }

    struct Policy {
        Duration max_snapshot_age{};
        Duration max_completion_span{};
        Duration max_observation_age{};
        Duration max_submission_age{};
        double max_tracking_error_rad = NAN;

        bool valid() const {
            return max_snapshot_age > Duration::zero() && max_completion_span > Duration::zero()
                && max_observation_age > Duration::zero() && max_submission_age > Duration::zero()
                && std::isfinite(max_tracking_error_rad) && max_tracking_error_rad > 0.0;
        }
    };

    struct AxisObservation {
        bool position_mode = false;
        bool closed_loop = false;
        bool error_free = false;
        double target_rad = NAN;
        double position_rad = NAN;
        Clock::time_point heartbeat_receipt{};
        Clock::time_point error_receipt{};
        Clock::time_point encoder_receipt{};
    };

    struct FrozenBatch {
        uint64_t activation_epoch = 0;
        uint64_t generation = 0;
        uint64_t snapshot_id = 0;
        bool position_hold_allowed = false;
        Clock::time_point source_time{};
        Clock::time_point batch_started{};
        size_t next_axis = 0;
        std::vector<std::optional<can_frame>> position_only_frames;
        std::vector<double> target_rad;
    };

    struct AxisReport {
        Level level = Level::unavailable;
        uint64_t generation = 0;
        uint64_t snapshot_id = 0;
        Clock::time_point submitted_at{};
        double tracking_error_rad = NAN;
    };

    struct Report {
        State state = State::disabled;
        uint64_t activation_epoch = 0;
        uint64_t generation = 0;
        uint64_t snapshot_id = 0;
        bool policy_ready = false;
        bool hold_confirmed = false;
        bool balance_verified = false;
        std::vector<AxisReport> axes;
    };

    enum class Decision { recover_hold, request_idle };

    void configure(Policy policy, size_t axes) {
        policy_ = policy;
        axes_ = axes;
        submitted_at_.assign(axes, Clock::time_point{});
        pending_axes_.assign(axes, {});
        complete_axes_.assign(axes, {});
        reset();
    }

    void activate(uint64_t activation_epoch) {
        reset();
        active_epoch_ = activation_epoch;
    }

    void reset() {
        failure_ = {};
        state_ = axes_ > 0 ? State::streaming : State::disabled;
        active_epoch_ = 0;
        batch_ = {};
        cursor_ = 0;
        entered_at_ = {};
        std::fill(submitted_at_.begin(), submitted_at_.end(), Clock::time_point{});
        pending_identity_ = {};
        complete_identity_ = {};
        std::fill(pending_axes_.begin(), pending_axes_.end(), StreamingAxis{});
        std::fill(complete_axes_.begin(), complete_axes_.end(), StreamingAxis{});
    }

    void request_stop() { state_ = State::stopping; }

    Decision on_batch_deadline(
        FrozenBatch batch,
        const std::vector<AxisObservation>& observations,
        Clock::time_point now,
        bool stop_requested
    ) {
        if (stop_requested) {
            record_failure(batch, now, "admission", "explicit_stop");
            request_stop();
            return Decision::request_idle;
        }
        if (!admissible(batch, observations, now)) {
            diagnose(batch, observations, now, "admission");
            state_ = State::unavailable;
            return Decision::request_idle;
        }
        batch_ = std::move(batch);
        cursor_ = batch_.next_axis;
        entered_at_ = now;
        state_ = cursor_ < axes_ ? State::finishing_batch : State::holding;
        return Decision::recover_hold;
    }

    std::optional<std::pair<size_t, can_frame>> next_frame(
        const std::vector<AxisObservation>& observations,
        Clock::time_point now,
        bool stop_requested
    ) {
        if (stop_requested) {
            record_failure(batch_, now, phase(), "explicit_stop");
            request_stop();
            return std::nullopt;
        }
        if ((state_ != State::finishing_batch && state_ != State::holding)
            || !observations_support_batch(observations, now) || !submissions_current(now)) {
            if (state_ == State::finishing_batch || state_ == State::holding) {
                diagnose(batch_, observations, now, phase(), false);
                diagnose_submissions(now);
                state_ = State::unavailable;
            }
            return std::nullopt;
        }
        if (state_ == State::finishing_batch && now - batch_.batch_started > policy_.max_completion_span) {
            record_failure(
                batch_,
                now,
                phase(),
                "completion_span_ms",
                no_axis,
                ms(now - batch_.batch_started),
                ms(policy_.max_completion_span)
            );
            state_ = State::unavailable;
            return std::nullopt;
        }
        for (size_t checked = 0; checked < axes_; ++checked) {
            if (cursor_ >= axes_) {
                state_ = State::holding;
                cursor_ = 0;
            }
            const size_t axis = cursor_++;
            if (batch_.position_only_frames[axis])
                return std::pair<size_t, can_frame>{axis, *batch_.position_only_frames[axis]};
        }
        record_failure(batch_, now, phase(), "missing_position_frame");
        state_ = State::unavailable;
        return std::nullopt;
    }

    void submitted(size_t axis, Clock::time_point now) {
        if (axis >= submitted_at_.size() || !submissions_current(now)
            || (state_ == State::finishing_batch
                && (now < batch_.batch_started || now - batch_.batch_started > policy_.max_completion_span))) {
            if (axis >= submitted_at_.size())
                record_failure(batch_, now, phase(), "axis_index", axis);
            diagnose_submissions(now);
            record_failure(
                batch_,
                now,
                phase(),
                "completion_span_ms",
                no_axis,
                ms(now - batch_.batch_started),
                ms(policy_.max_completion_span)
            );
            state_ = State::unavailable;
            return;
        }
        submitted_at_[axis] = now;
        if (state_ == State::finishing_batch && cursor_ >= axes_) {
            state_ = State::holding;
            cursor_ = 0;
        }
    }

    void submission_failed() {
        record_failure(batch_, Clock::now(), phase(), "transport_send_failure");
        if (state_ != State::stopping)
            state_ = State::unavailable;
    }

    void streaming_submitted(
        size_t axis,
        uint64_t activation_epoch,
        uint64_t generation,
        uint64_t snapshot_id,
        double target_rad,
        Clock::time_point now
    ) {
        if (state_ != State::streaming || axis >= axes_ || activation_epoch == 0 || generation == 0 || snapshot_id == 0
            || !std::isfinite(target_rad))
            return;
        const Identity identity{activation_epoch, generation, snapshot_id};
        if (!(pending_identity_ == identity)) {
            pending_identity_ = identity;
            std::fill(pending_axes_.begin(), pending_axes_.end(), StreamingAxis{});
        }
        pending_axes_[axis] = {true, now, target_rad};
        if (std::all_of(pending_axes_.begin(), pending_axes_.end(), [](const auto& row) { return row.submitted; })) {
            complete_identity_ = pending_identity_;
            complete_axes_ = pending_axes_;
        }
    }

    bool requires_idle() const { return state_ == State::unavailable || state_ == State::stopping; }

    State state() const { return state_; }

    Report report(const std::vector<AxisObservation>& observations, Clock::time_point now) const {
        Report result;
        result.state = state_;
        result.policy_ready = policy_.valid();
        if (state_ == State::streaming) {
            result.activation_epoch = complete_identity_.activation_epoch;
            result.generation = complete_identity_.generation;
            result.snapshot_id = complete_identity_.snapshot_id;
            result.axes.resize(axes_);
            result.hold_confirmed = axes_ > 0 && complete_identity_.activation_epoch == active_epoch_
                                 && complete_identity_.generation != 0 && complete_identity_.snapshot_id != 0
                                 && observations.size() == axes_ && policy_.valid();
            for (size_t axis = 0; axis < axes_; ++axis) {
                auto& row = result.axes[axis];
                const auto& submitted = complete_axes_[axis];
                row.generation = complete_identity_.generation;
                row.snapshot_id = complete_identity_.snapshot_id;
                row.submitted_at = submitted.submitted_at;
                row.tracking_error_rad = axis < observations.size() && submitted.submitted
                                           ? std::abs(submitted.target_rad - observations[axis].position_rad)
                                           : NAN;
                const bool current_submission = submitted.submitted && now >= submitted.submitted_at
                                             && now - submitted.submitted_at <= policy_.max_submission_age;
                if (!submitted.submitted) {
                    row.level = Level::unavailable;
                } else if (policy_.valid() && current_submission
                           && axis < observations.size()
                           && observation_current(observations[axis], now)
                           && row.tracking_error_rad <= policy_.max_tracking_error_rad) {
                    row.level = Level::observed;
                } else {
                    row.level = Level::submitted;
                }
                result.hold_confirmed &= row.level == Level::observed;
            }
            return result;
        }
        result.activation_epoch = batch_.activation_epoch;
        result.generation = batch_.generation;
        result.snapshot_id = batch_.snapshot_id;
        result.axes.resize(axes_);
        result.hold_confirmed = axes_ > 0 && (state_ == State::holding || state_ == State::finishing_batch)
                             && observations.size() == axes_;
        for (size_t axis = 0; axis < axes_; ++axis) {
            auto& row = result.axes[axis];
            row.generation = batch_.generation;
            row.snapshot_id = batch_.snapshot_id;
            row.submitted_at = submitted_at_[axis];
            if (axis >= observations.size()) {
                row.level = Level::unavailable;
                result.hold_confirmed = false;
                continue;
            }
            const auto& observation = observations[axis];
            row.tracking_error_rad = axis < batch_.target_rad.size()
                                       ? std::abs(batch_.target_rad[axis] - observation.position_rad)
                                       : NAN;
            const bool submitted = submitted_at_[axis] != Clock::time_point{} && now >= submitted_at_[axis]
                                && now - submitted_at_[axis] <= policy_.max_submission_age;
            if (!submitted) {
                row.level = state_ == State::finishing_batch || state_ == State::holding ? Level::requested
                                                                                         : Level::unavailable;
            } else if (observation_current(observation, now) && row.tracking_error_rad <= policy_.max_tracking_error_rad) {
                row.level = Level::observed;
            } else {
                row.level = Level::submitted;
            }
            result.hold_confirmed &= row.level == Level::observed;
        }
        return result;
    }

private:
    static constexpr size_t no_axis = static_cast<size_t>(-1);
    static double ms(Duration value) { return std::chrono::duration<double, std::milli>(value).count(); }
    const char* phase() const {
        return state_ == State::finishing_batch ? "finishing_batch"
             : state_ == State::holding         ? "holding"
                                                : "streaming";
    }
    void record_failure(
        const FrozenBatch& batch,
        Clock::time_point now,
        const char* phase_name,
        const char* check,
        size_t axis = no_axis,
        double actual = NAN,
        double limit = NAN,
        double position = NAN
    ) {
        if (failure_.check != std::string_view("none"))
            return;
        failure_ = {
            phase_name,
            check,
            axis,
            actual,
            limit,
            axis < batch.target_rad.size() ? batch.target_rad[axis] : NAN,
            position,
            ms(now - batch.source_time),
            ms(now - batch.batch_started),
            batch.generation,
            batch.snapshot_id,
            batch.activation_epoch};
    }
    void diagnose_submissions(Clock::time_point now) {
        for (size_t axis = 0; axis < submitted_at_.size(); ++axis) {
            const auto since = submitted_at_[axis] == Clock::time_point{} ? entered_at_ : submitted_at_[axis];
            if (now < since || now - since > policy_.max_submission_age) {
                record_failure(
                    batch_,
                    now,
                    phase(),
                    "submission_age_ms",
                    axis,
                    ms(now - since),
                    ms(policy_.max_submission_age)
                );
                return;
            }
        }
    }
    // Called only after the existing decision rejects support. This does not
    // change eligibility or evaluate logging/formatting in the normal send loop.
    void diagnose(
        const FrozenBatch& batch,
        const std::vector<AxisObservation>& obs,
        Clock::time_point now,
        const char* stage,
        bool admission = true
    ) {
        const auto fail = [&](const char* check, size_t axis = no_axis, double actual = NAN, double limit = NAN) {
            record_failure(
                batch,
                now,
                stage,
                check,
                axis,
                actual,
                limit,
                axis < obs.size() ? obs[axis].position_rad : NAN
            );
        };
        if (admission) {
            if (!policy_.valid()) {
                fail("policy_invalid");
                return;
            }
            if (!axes_ || !batch.position_hold_allowed) {
                fail("position_hold_not_allowed");
                return;
            }
            if (!active_epoch_ || batch.activation_epoch != active_epoch_ || !batch.generation) {
                fail("batch_identity");
                return;
            }
            if (batch.position_only_frames.size() != axes_ || batch.next_axis > axes_) {
                fail("batch_shape");
                return;
            }
            for (size_t i = 0; i < axes_; ++i)
                if (!batch.position_only_frames[i]) {
                    fail("missing_position_frame", i);
                    return;
                }
        }
        if (batch.target_rad.size() != axes_ || obs.size() != axes_) {
            fail("observation_shape");
            return;
        }
        for (size_t i = 0; i < axes_; ++i)
            if (!std::isfinite(batch.target_rad[i])) {
                fail("target_nonfinite", i);
                return;
            }
        if (admission) {
            if (now < batch.source_time || now - batch.source_time > policy_.max_snapshot_age) {
                fail("snapshot_age_ms", no_axis, ms(now - batch.source_time), ms(policy_.max_snapshot_age));
                return;
            }
            if (now < batch.batch_started || now - batch.batch_started > policy_.max_completion_span) {
                fail("completion_span_ms", no_axis, ms(now - batch.batch_started), ms(policy_.max_completion_span));
                return;
            }
        }
        for (size_t i = 0; i < axes_; ++i) {
            const auto& row = obs[i];
            if (!row.position_mode) {
                fail("position_mode", i, 0, 1);
                return;
            }
            if (!row.closed_loop) {
                fail("closed_loop", i, 0, 1);
                return;
            }
            if (!row.error_free) {
                fail("motor_error", i, 1, 0);
                return;
            }
            if (!std::isfinite(row.position_rad)) {
                fail("position_nonfinite", i);
                return;
            }
            for (const auto& entry :
                 {std::pair{"heartbeat_age_ms", row.heartbeat_receipt},
                  std::pair{"error_age_ms", row.error_receipt},
                  std::pair{"encoder_age_ms", row.encoder_receipt}}) {
                if (entry.second == Clock::time_point{} || now < entry.second
                    || now - entry.second > policy_.max_observation_age) {
                    fail(
                        entry.first,
                        i,
                        entry.second == Clock::time_point{} ? NAN : ms(now - entry.second),
                        ms(policy_.max_observation_age)
                    );
                    return;
                }
            }
            const double error = std::abs(batch.target_rad[i] - row.position_rad);
            if (error > policy_.max_tracking_error_rad) {
                fail("tracking_error_rad", i, error, policy_.max_tracking_error_rad);
                return;
            }
        }
    }
    Failure failure_{};
    bool submissions_current(Clock::time_point now) const {
        // Hold has no total-duration timeout. Each axis must still receive
        // timely submissions; fresh feedback alone cannot hide a stalled writer.
        return std::all_of(submitted_at_.begin(), submitted_at_.end(), [&](const auto stamp) {
            const auto since = stamp == Clock::time_point{} ? entered_at_ : stamp;
            return now >= since && now - since <= policy_.max_submission_age;
        });
    }

    bool observation_current(const AxisObservation& value, Clock::time_point now) const {
        const auto current = [&](Clock::time_point receipt) {
            return receipt != Clock::time_point{} && now >= receipt && now - receipt <= policy_.max_observation_age;
        };
        return value.position_mode && value.closed_loop && value.error_free && std::isfinite(value.position_rad)
            && current(value.heartbeat_receipt) && current(value.error_receipt) && current(value.encoder_receipt);
    }

    bool observations_current(const std::vector<AxisObservation>& observations, Clock::time_point now) const {
        return observations.size() == axes_
            && std::all_of(observations.begin(), observations.end(), [&](const auto& row) {
                   return observation_current(row, now);
               });
    }

    bool observations_support_batch(const std::vector<AxisObservation>& observations, Clock::time_point now) const {
        if (!observations_current(observations, now) || batch_.target_rad.size() != axes_)
            return false;
        for (size_t axis = 0; axis < axes_; ++axis) {
            if (!std::isfinite(batch_.target_rad[axis])
                || std::abs(batch_.target_rad[axis] - observations[axis].position_rad) > policy_.max_tracking_error_rad)
                return false;
        }
        return true;
    }

    bool admissible(const FrozenBatch& batch, const std::vector<AxisObservation>& observations, Clock::time_point now)
        const {
        return policy_.valid() && axes_ > 0 && batch.position_hold_allowed && active_epoch_ != 0
            && batch.activation_epoch == active_epoch_ && batch.generation != 0
            && batch.position_only_frames.size() == axes_ && batch.next_axis <= axes_
            && std::all_of(
                   batch.position_only_frames.begin(),
                   batch.position_only_frames.end(),
                   [](const auto& frame) { return frame.has_value(); }
            )
            && batch.target_rad.size() == axes_ && observations.size() == axes_
            && std::all_of(
                   batch.target_rad.begin(),
                   batch.target_rad.end(),
                   [](double value) { return std::isfinite(value); }
            )
            && now >= batch.source_time && now - batch.source_time <= policy_.max_snapshot_age
            && now >= batch.batch_started && now - batch.batch_started <= policy_.max_completion_span
            && std::equal(
                   batch.target_rad.begin(),
                   batch.target_rad.end(),
                   observations.begin(),
                   [&](double target, const auto& observation) {
            return observation_current(observation, now)
                && std::abs(target - observation.position_rad) <= policy_.max_tracking_error_rad;
                   });
    }

    struct Identity {
        uint64_t activation_epoch = 0;
        uint64_t generation = 0;
        uint64_t snapshot_id = 0;
        bool operator==(const Identity&) const = default;
    };

    struct StreamingAxis {
        bool submitted = false;
        Clock::time_point submitted_at{};
        double target_rad = NAN;
    };

    Policy policy_{};
    uint64_t active_epoch_ = 0;
    size_t axes_ = 0;
    State state_ = State::disabled;
    FrozenBatch batch_{};
    size_t cursor_ = 0;
    Clock::time_point entered_at_{};
    std::vector<Clock::time_point> submitted_at_;
    Identity pending_identity_{}, complete_identity_{};
    std::vector<StreamingAxis> pending_axes_, complete_axes_;
};

} // namespace odrive_ros2_control

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <linux/can.h>
#include <optional>
#include <utility>
#include <vector>

namespace odrive_ros2_control {

class SupportFallback {
public:
    using Clock = std::chrono::steady_clock;
    using Duration = Clock::duration;

    enum class State { disabled, streaming, finishing_batch, holding, unavailable, stopping };
    enum class Level { requested, submitted, observed, unavailable };

    struct Policy {
        Duration max_snapshot_age{};
        Duration max_completion_span{};
        Duration max_observation_age{};
        Duration max_submission_age{};
        Duration max_hold_duration{};
        double max_tracking_error_rad = NAN;

        bool valid() const {
            return max_snapshot_age > Duration::zero()
                && max_completion_span > Duration::zero()
                && max_observation_age > Duration::zero()
                && max_submission_age > Duration::zero()
                && max_hold_duration > Duration::zero()
                && std::isfinite(max_tracking_error_rad)
                && max_tracking_error_rad > 0.0;
        }
    };

    struct Authority {
        uint64_t activation_epoch = 0;
        uint64_t generation = 0;
        bool position_hold_allowed = false;
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
        Clock::time_point source_time{};
        Clock::time_point batch_started{};
        size_t next_axis = 0;
        std::vector<std::optional<can_frame>> position_only_frames;
        std::vector<double> target_rad;
    };

    struct AxisReport {
        Level level = Level::unavailable;
        Clock::time_point submitted_at{};
        double tracking_error_rad = NAN;
    };

    struct Report {
        State state = State::disabled;
        uint64_t activation_epoch = 0;
        uint64_t generation = 0;
        uint64_t snapshot_id = 0;
        bool hold_confirmed = false;
        bool balance_verified = false;
        std::vector<AxisReport> axes;
    };

    enum class Decision { recover_hold, request_idle };

    void configure(Policy policy, size_t axes) {
        policy_ = policy;
        axes_ = axes;
        submitted_at_.assign(axes, Clock::time_point{});
        reset();
    }

    void authorize(Authority authority) { authority_ = authority; }

    void reset() {
        state_ = policy_.valid() && axes_ > 0 ? State::streaming : State::disabled;
        batch_ = {};
        cursor_ = 0;
        entered_at_ = {};
        std::fill(submitted_at_.begin(), submitted_at_.end(), Clock::time_point{});
    }

    void request_stop() { state_ = State::stopping; }

    Decision on_batch_deadline(
        FrozenBatch batch,
        const std::vector<AxisObservation>& observations,
        Clock::time_point now,
        bool stop_requested
    ) {
        if (stop_requested) {
            request_stop();
            return Decision::request_idle;
        }
        if (!admissible(batch, observations, now)) {
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
            request_stop();
            return std::nullopt;
        }
        if ((state_ != State::finishing_batch && state_ != State::holding)
            || !observations_support_batch(observations, now)
            || now - entered_at_ > policy_.max_hold_duration) {
            if (state_ == State::finishing_batch || state_ == State::holding)
                state_ = State::unavailable;
            return std::nullopt;
        }
        if (state_ == State::finishing_batch
            && now - batch_.batch_started > policy_.max_completion_span) {
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
        state_ = State::unavailable;
        return std::nullopt;
    }

    void submitted(size_t axis, Clock::time_point now) {
        if (axis >= submitted_at_.size()) {
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
        if (state_ != State::stopping) state_ = State::unavailable;
    }

    bool requires_idle() const {
        return state_ == State::unavailable || state_ == State::stopping;
    }

    State state() const { return state_; }

    Report report(const std::vector<AxisObservation>& observations, Clock::time_point now) const {
        Report result;
        result.state = state_;
        result.activation_epoch = batch_.activation_epoch;
        result.generation = batch_.generation;
        result.snapshot_id = batch_.snapshot_id;
        result.axes.resize(axes_);
        result.hold_confirmed = axes_ > 0
            && (state_ == State::holding || state_ == State::finishing_batch)
            && observations.size() == axes_;
        for (size_t axis = 0; axis < axes_; ++axis) {
            auto& row = result.axes[axis];
            row.submitted_at = submitted_at_[axis];
            if (axis >= observations.size()) {
                row.level = Level::unavailable;
                result.hold_confirmed = false;
                continue;
            }
            const auto& observation = observations[axis];
            row.tracking_error_rad = axis < batch_.target_rad.size()
                ? std::abs(batch_.target_rad[axis] - observation.position_rad) : NAN;
            const bool submitted = submitted_at_[axis] != Clock::time_point{}
                && now >= submitted_at_[axis]
                && now - submitted_at_[axis] <= policy_.max_submission_age;
            if (!submitted) {
                row.level = state_ == State::finishing_batch || state_ == State::holding
                    ? Level::requested : Level::unavailable;
            } else if (observation_current(observation, now)
                       && row.tracking_error_rad <= policy_.max_tracking_error_rad) {
                row.level = Level::observed;
            } else {
                row.level = Level::submitted;
            }
            result.hold_confirmed &= row.level == Level::observed;
        }
        return result;
    }

private:
    bool observation_current(const AxisObservation& value, Clock::time_point now) const {
        const auto current = [&](Clock::time_point receipt) {
            return receipt != Clock::time_point{} && now >= receipt
                && now - receipt <= policy_.max_observation_age;
        };
        return value.position_mode && value.closed_loop && value.error_free
            && std::isfinite(value.position_rad)
            && current(value.heartbeat_receipt) && current(value.error_receipt)
            && current(value.encoder_receipt);
    }

    bool observations_current(
        const std::vector<AxisObservation>& observations, Clock::time_point now
    ) const {
        return observations.size() == axes_
            && std::all_of(observations.begin(), observations.end(),
                           [&](const auto& row) { return observation_current(row, now); });
    }

    bool observations_support_batch(
        const std::vector<AxisObservation>& observations, Clock::time_point now
    ) const {
        if (!observations_current(observations, now) || batch_.target_rad.size() != axes_)
            return false;
        for (size_t axis = 0; axis < axes_; ++axis) {
            if (!std::isfinite(batch_.target_rad[axis])
                || std::abs(batch_.target_rad[axis] - observations[axis].position_rad)
                    > policy_.max_tracking_error_rad)
                return false;
        }
        return true;
    }

    bool admissible(
        const FrozenBatch& batch,
        const std::vector<AxisObservation>& observations,
        Clock::time_point now
    ) const {
        return policy_.valid() && axes_ > 0 && authority_.position_hold_allowed
            && authority_.activation_epoch != 0 && authority_.generation != 0
            && batch.activation_epoch == authority_.activation_epoch
            && batch.generation != 0 && batch.generation == authority_.generation
            && batch.position_only_frames.size() == axes_ && batch.next_axis <= axes_
            && std::all_of(batch.position_only_frames.begin(), batch.position_only_frames.end(),
                           [](const auto& frame) { return frame.has_value(); })
            && batch.target_rad.size() == axes_
            && observations.size() == axes_
            && std::all_of(batch.target_rad.begin(), batch.target_rad.end(),
                           [](double value) { return std::isfinite(value); })
            && now >= batch.source_time && now - batch.source_time <= policy_.max_snapshot_age
            && now >= batch.batch_started
            && now - batch.batch_started <= policy_.max_completion_span
            && std::equal(
                batch.target_rad.begin(), batch.target_rad.end(), observations.begin(),
                [&](double target, const auto& observation) {
                    return observation_current(observation, now)
                        && std::abs(target - observation.position_rad)
                            <= policy_.max_tracking_error_rad;
                });
    }

    Policy policy_{};
    Authority authority_{};
    size_t axes_ = 0;
    State state_ = State::disabled;
    FrozenBatch batch_{};
    size_t cursor_ = 0;
    Clock::time_point entered_at_{};
    std::vector<Clock::time_point> submitted_at_;
};

}  // namespace odrive_ros2_control

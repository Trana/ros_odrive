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

    void streaming_submitted(
        size_t axis,
        uint64_t activation_epoch,
        uint64_t generation,
        uint64_t snapshot_id,
        double target_rad,
        Clock::time_point now
    ) {
        if (state_ != State::streaming || axis >= axes_ || activation_epoch == 0
            || generation == 0 || snapshot_id == 0 || !std::isfinite(target_rad))
            return;
        const Identity identity{activation_epoch, generation, snapshot_id};
        if (!(pending_identity_ == identity)) {
            pending_identity_ = identity;
            std::fill(pending_axes_.begin(), pending_axes_.end(), StreamingAxis{});
        }
        pending_axes_[axis] = {true, now, target_rad};
        if (std::all_of(pending_axes_.begin(), pending_axes_.end(),
                        [](const auto& row) { return row.submitted; })) {
            complete_identity_ = pending_identity_;
            complete_axes_ = pending_axes_;
        }
    }

    bool requires_idle() const {
        return state_ == State::unavailable || state_ == State::stopping;
    }

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
            result.hold_confirmed = axes_ > 0
                && complete_identity_.activation_epoch == active_epoch_
                && complete_identity_.generation != 0
                && complete_identity_.snapshot_id != 0
                && observations.size() == axes_ && policy_.valid();
            for (size_t axis = 0; axis < axes_; ++axis) {
                auto& row = result.axes[axis];
                const auto& submitted = complete_axes_[axis];
                row.generation = complete_identity_.generation;
                row.snapshot_id = complete_identity_.snapshot_id;
                row.submitted_at = submitted.submitted_at;
                row.tracking_error_rad = axis < observations.size() && submitted.submitted
                    ? std::abs(submitted.target_rad - observations[axis].position_rad) : NAN;
                const bool current_submission = submitted.submitted
                    && now >= submitted.submitted_at
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
        result.hold_confirmed = axes_ > 0
            && (state_ == State::holding || state_ == State::finishing_batch)
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
        return policy_.valid() && axes_ > 0 && batch.position_hold_allowed
            && active_epoch_ != 0 && batch.activation_epoch == active_epoch_
            && batch.generation != 0
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

}  // namespace odrive_ros2_control

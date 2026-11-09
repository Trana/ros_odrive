#pragma once

#include "hardware_timing.hpp"
#include "support_fallback.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <linux/can.h>
#include <optional>
#include <stdexcept>
#include <vector>

namespace odrive_ros2_control {

// Caller serializes access, including the actual send, with its transport mutex.
// Fixed slot per axis: replacements never queue historical commands.
class PacedCommands {
public:
    using Clock = std::chrono::steady_clock;
    struct DueFrame {
        can_frame frame{};
        size_t axis = 0;
        uint64_t activation_epoch = 0;
        uint64_t generation = 0;
        uint64_t snapshot_id = 0;
        double hold_target_rad = NAN;
    };
    enum class FaultReason { none, producer_stale, batch_deadline, snapshot_stale, non_finite_target, send_failure };
    FaultReason fault_reason() const { return fault_reason_; }
    const char* fault_reason_name() const {
        switch (fault_reason_) {
            case FaultReason::none: return "none";
            case FaultReason::producer_stale: return "producer_stale";
            case FaultReason::batch_deadline: return "batch_deadline";
            case FaultReason::snapshot_stale: return "snapshot_stale";
            case FaultReason::non_finite_target: return "non_finite_target";
            case FaultReason::send_failure: return "send_failure";
        }
        return "unknown";
    }
    void set_batch_timing(HardwareTiming::BatchTiming* timing) { batch_timing_ = timing; }
    void configure(
        size_t axes,
        double rate_hz,
        double stale_ms,
        bool phase_locked = false,
        bool bounded_phase = false,
        bool coherent_batch = false,
        double batch_gap_us = 0
    ) {
        if (axes == 0 || axes > 64 || !std::isfinite(rate_hz) || rate_hz < 1 || rate_hz > 200
            || !std::isfinite(stale_ms) || stale_ms < 20 || stale_ms > 1000)
            throw std::invalid_argument("Invalid CAN pacing axes/rate/stale limit");
        if (!std::isfinite(batch_gap_us) || batch_gap_us < 0 || batch_gap_us > 1000
            || (!coherent_batch && batch_gap_us != 0) || batch_gap_us * (axes - 1) >= 1e6 / rate_hz)
            throw std::invalid_argument("Invalid CAN batch gap or gap budget exceeds round period");
        frames_.resize(axes);
        batch_frames_.resize(axes);
        hold_frames_.resize(axes);
        batch_hold_frames_.resize(axes);
        hold_targets_.resize(axes, NAN);
        batch_hold_targets_.resize(axes, NAN);
        batch_mode_ = coherent_batch;
        batch_gap_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::micro>(batch_gap_us)
        );
        period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / rate_hz));
        phase_locked_ = phase_locked || bounded_phase;
        bounded_phase_ = bounded_phase;
        slot_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / (rate_hz * axes)));
        stale_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(stale_ms));
        inhibit();
    }
    void discard() {
        if (batch_pending_ && batch_timing_)
            batch_timing_->aborted.fetch_add(1);
        batch_pending_ = false;
        fresh_ = false;
        batch_active_ = false;
        for (auto& frame : frames_)
            frame.reset();
    }
    void inhibit() {
        enabled_ = false;
        discard();
    }
    void activate(Clock::time_point now) {
        inhibit();
        enabled_ = true;
        fault_ = false;
        fault_reason_ = FaultReason::none;
        cursor_ = 0;
        next_ = now;
    }
    // Preserve the first cause through inhibition/shutdown and later failures.
    void fault(FaultReason reason) {
        if (!fault_)
            fault_reason_ = reason;
        inhibit();
        fault_ = true;
    }
    bool faulted() const { return fault_; }
    void clear_fault_after_idle() {
        fault_ = false;
        fault_reason_ = FaultReason::none;
    }
    void set(size_t axis, std::optional<can_frame> frame) { set(axis, frame, frame, 0.0); }
    void set(
        size_t axis,
        std::optional<can_frame> frame,
        std::optional<can_frame> position_only_frame,
        double target_rad
    ) {
        frames_.at(axis) = frame;
        hold_frames_.at(axis) = position_only_frame;
        hold_targets_.at(axis) = target_rad;
    }
    void commit(Clock::time_point now) {
        ++implicit_generation_;
        commit(now, 1, implicit_generation_, implicit_generation_);
    }
    void commit(
        Clock::time_point now,
        uint64_t activation_epoch,
        uint64_t generation,
        uint64_t snapshot_id,
        bool position_hold_allowed = false
    ) {
        if (enabled_) {
            updated_ = now;
            activation_epoch_ = activation_epoch;
            generation_ = generation;
            snapshot_id_ = snapshot_id;
            position_hold_allowed_ = position_hold_allowed;
            fresh_ = true;
        }
    }
    std::optional<DueFrame> due(Clock::time_point now) {
        if (!enabled_ || !fresh_)
            return {};
        if (now - updated_ >= stale_) {
            fault(FaultReason::producer_stale);
            return {};
        }
        if (now < next_)
            return {};
        if (batch_mode_) {
            if (!batch_active_) {
                // Preallocated storage; one hardware-write generation per round.
                std::copy(frames_.begin(), frames_.end(), batch_frames_.begin());
                std::copy(hold_frames_.begin(), hold_frames_.end(), batch_hold_frames_.begin());
                std::copy(hold_targets_.begin(), hold_targets_.end(), batch_hold_targets_.begin());
                batch_updated_ = updated_;
                batch_started_ = now;
                batch_activation_epoch_ = activation_epoch_;
                batch_generation_ = generation_;
                batch_snapshot_id_ = snapshot_id_;
                batch_position_hold_allowed_ = position_hold_allowed_;
                cursor_ = 0;
                batch_active_ = true;
                batch_pending_ = true;
                if (batch_timing_)
                    batch_timing_->started.fetch_add(1);
            }
            // A fresh producer cannot keep an old partial batch alive indefinitely.
            if (now - batch_started_ >= period_) {
                if (batch_pending_ && batch_timing_)
                    batch_timing_->aborted.fetch_add(1);
                batch_pending_ = false;
                if (!fault_)
                    fault_reason_ = FaultReason::batch_deadline;
                enabled_ = false;
                fresh_ = false;
                fault_ = true;
                return {};
            }
            if (now - batch_updated_ >= stale_) {
                fault(FaultReason::snapshot_stale);
                return {};
            }
            const size_t axis = cursor_++;
            auto frame = batch_frames_[axis];
            if (cursor_ == frames_.size()) {
                batch_active_ = false;
                next_ = batch_started_ + period_;
            } else {
                next_ = now + batch_gap_; // Completion extends this deadline below.
            }
            if (!frame)
                return {};
            return DueFrame{
                *frame,
                axis,
                batch_activation_epoch_,
                batch_generation_,
                batch_snapshot_id_,
                batch_hold_targets_[axis]};
        }
        // Never replay missed slots or emit a catch-up burst after a stall.
        next_ = phase_locked_ ? phase_deadline(next_ + slot_, now + slot_ * 9 / 10) : now + slot_;
        const size_t axis = cursor_;
        auto frame = frames_[axis];
        cursor_ = (cursor_ + 1) % frames_.size();
        if (!frame)
            return {};
        return DueFrame{*frame, axis, activation_epoch_, generation_, snapshot_id_, hold_targets_[axis]};
    }
    Clock::duration slot() const { return slot_; }
    std::optional<SupportFallback::FrozenBatch> take_recovery_batch() {
        if (!fault_ || fault_reason_ != FaultReason::batch_deadline || !batch_active_)
            return std::nullopt;
        SupportFallback::FrozenBatch result;
        result.activation_epoch = batch_activation_epoch_;
        result.generation = batch_generation_;
        result.snapshot_id = batch_snapshot_id_;
        result.position_hold_allowed = batch_position_hold_allowed_;
        result.source_time = batch_updated_;
        result.batch_started = batch_started_;
        result.next_axis = cursor_;
        result.position_only_frames = batch_hold_frames_;
        result.target_rad = batch_hold_targets_;
        batch_pending_ = false;
        batch_active_ = false;
        return result;
    }
    // Called once after each due() opportunity under the transport mutex.
    // A completed batch means socket acceptance, not motor acknowledgement.
    void finish_submission(Clock::time_point now, bool submitted_frame) {
        if (!batch_mode_ || !batch_pending_)
            return;
        if (batch_timing_ && submitted_frame)
            batch_timing_->submitted_frames.fetch_add(1);
        if (batch_active_)
            next_ = now + batch_gap_;
        if (!batch_active_) {
            if (batch_timing_) {
                batch_timing_->completed.fetch_add(1);
                batch_timing_->submission_span.add(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now - batch_started_).count()
                );
            }
            batch_pending_ = false;
        }
    }
    // Only meaningful after due() returned a frame. This is hardware mailbox
    // age, not policy-action age: repeated hardware writes refresh updated_.
    Clock::duration target_age(Clock::time_point now) const { return now - (batch_mode_ ? batch_updated_ : updated_); }
    // Deadline measured from the send opportunity, not from send completion.
    // If work overran it, skip the slot instead of immediately catching up.
    Clock::time_point wake_after(Clock::time_point completed) const {
        if (batch_mode_ && enabled_ && fresh_) {
            if (batch_active_)
                return std::max(next_, completed);
            return next_ > completed ? next_ : completed + period_;
        }
        if (enabled_ && fresh_ && next_ > completed)
            return next_;
        return phase_locked_ && enabled_ && fresh_ ? phase_deadline(next_, completed + slot_) : completed + slot_;
    }

private:
    // Constant-time slot skipping; never loop over or replay missed commands.
    Clock::time_point phase_deadline(Clock::time_point grid, Clock::time_point earliest) const {
        // Re-anchor only enough to satisfy the spacing floor. No slot replay.
        if (bounded_phase_)
            return std::max(grid, earliest);
        if (grid >= earliest)
            return grid;
        const auto gap = earliest - grid;
        const auto slots = gap / slot_ + (gap % slot_ != Clock::duration::zero());
        return grid + slots * slot_;
    }
    std::vector<std::optional<can_frame>> frames_;
    std::vector<std::optional<can_frame>> batch_frames_;
    std::vector<std::optional<can_frame>> hold_frames_;
    std::vector<std::optional<can_frame>> batch_hold_frames_;
    std::vector<double> hold_targets_;
    std::vector<double> batch_hold_targets_;
    Clock::duration period_{};
    Clock::duration batch_gap_{};
    Clock::time_point batch_started_{}, batch_updated_{};
    bool batch_mode_ = false, batch_active_ = false;
    bool batch_pending_ = false;
    HardwareTiming::BatchTiming* batch_timing_ = nullptr;
    Clock::duration slot_{}, stale_{};
    Clock::time_point next_{}, updated_{};
    size_t cursor_ = 0;
    uint64_t implicit_generation_ = 0;
    uint64_t activation_epoch_ = 0, generation_ = 0, snapshot_id_ = 0;
    uint64_t batch_activation_epoch_ = 0, batch_generation_ = 0, batch_snapshot_id_ = 0;
    bool position_hold_allowed_ = false, batch_position_hold_allowed_ = false;
    bool enabled_ = false, fresh_ = false, fault_ = false;
    FaultReason fault_reason_ = FaultReason::none;
    bool phase_locked_ = false;
    bool bounded_phase_ = false;
};
} // namespace odrive_ros2_control

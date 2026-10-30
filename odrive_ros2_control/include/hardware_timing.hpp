#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <time.h>

namespace odrive_ros2_control {

// Fixed-size diagnostic counters. No publishing, allocation or waiting in add().
// Export boundaries are approximate: count/sum/max are separate atomic values.
struct DurationCounter {
    std::atomic<uint64_t> count{0}, sum_ns{0}, max_ns{0};

    void add(uint64_t ns) {
        count.fetch_add(1, std::memory_order_relaxed);
        sum_ns.fetch_add(ns, std::memory_order_relaxed);
        auto maximum = max_ns.load(std::memory_order_relaxed);
        if (ns > maximum) {
            // Single sample writer; the exporter may concurrently reset maximum.
            // At most two attempts, never an unbounded retry in the control loop.
            if (!max_ns.compare_exchange_strong(maximum, ns, std::memory_order_relaxed)
                && ns > maximum) {
                max_ns.compare_exchange_strong(maximum, ns, std::memory_order_relaxed);
            }
        }
    }

    void json(std::ostream& out) {
        const auto n = count.exchange(0);
        const auto total = sum_ns.exchange(0);
        const auto maximum = max_ns.exchange(0);
        out << "{\"count\":" << n << ",\"mean_ms\":"
            << (n ? static_cast<double>(total) / n / 1e6 : 0.0)
            << ",\"max_ms\":" << maximum / 1e6 << '}';
    }
};

class TimedScope {
public:
    using Clock = std::chrono::steady_clock;
    explicit TimedScope(DurationCounter* counter) : counter_(counter) {
        if (counter_) start_ = Clock::now();
    }
    ~TimedScope() {
        if (counter_) counter_->add(std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - start_).count());
    }
    TimedScope(const TimedScope&) = delete;
    TimedScope& operator=(const TimedScope&) = delete;
private:
    DurationCounter* counter_;
    Clock::time_point start_;
};

// Paired samples: never subtract independently exported maxima. CPU clocks are
// diagnostic only; failure must not affect transport or look like zero CPU work.
struct SendExecutionTiming {
    DurationCounter elapsed, cpu, off_cpu;
    DurationCounter slow_elapsed, slow_cpu, slow_off_cpu;
    std::atomic<uint64_t> invalid_cpu_samples{0};
    void add(uint64_t wall_ns, uint64_t cpu_ns, bool valid) {
        elapsed.add(wall_ns);
        if (!valid || cpu_ns > wall_ns) {
            invalid_cpu_samples.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        cpu.add(cpu_ns);
        off_cpu.add(wall_ns - cpu_ns);
        if (wall_ns >= 2000000) {
            slow_elapsed.add(wall_ns);
            slow_cpu.add(cpu_ns);
            slow_off_cpu.add(wall_ns - cpu_ns);
        }
    }
    void json(std::ostream& out) {
        out << "{\"slow_threshold_ms\":2,\"elapsed\":"; elapsed.json(out);
        out << ",\"cpu\":"; cpu.json(out);
        out << ",\"off_cpu_estimate\":"; off_cpu.json(out);
        out << ",\"slow_elapsed\":"; slow_elapsed.json(out);
        out << ",\"slow_cpu\":"; slow_cpu.json(out);
        out << ",\"slow_off_cpu_estimate\":"; slow_off_cpu.json(out);
        out << ",\"invalid_cpu_samples\":" << invalid_cpu_samples.exchange(0) << '}';
    }
};

class SendExecutionScope {
public:
    explicit SendExecutionScope(SendExecutionTiming* timing) : timing_(timing) {
        if (timing_) {
            wall_start_ = std::chrono::steady_clock::now();
            valid_ = thread_cpu(cpu_start_);
        }
    }
    ~SendExecutionScope() {
        if (!timing_) return;
        uint64_t cpu_end = 0;
        const bool valid = thread_cpu(cpu_end) && valid_ && cpu_end >= cpu_start_;
        const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - wall_start_).count();
        timing_->add(static_cast<uint64_t>(wall), valid ? cpu_end - cpu_start_ : 0, valid);
    }
    SendExecutionScope(const SendExecutionScope&) = delete;
    SendExecutionScope& operator=(const SendExecutionScope&) = delete;
private:
    static bool thread_cpu(uint64_t& ns) {
        timespec value{};
        if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) return false;
        ns = static_cast<uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
        return true;
    }
    SendExecutionTiming* timing_;
    std::chrono::steady_clock::time_point wall_start_{};
    uint64_t cpu_start_ = 0;
    bool valid_ = false;
};

struct HardwareTiming {
    struct BatchTiming {
        DurationCounter submission_span;
        std::atomic<uint64_t> started{0}, completed{0}, aborted{0}, submitted_frames{0};
        void json(std::ostream& out) {
            out << "{\"started\":" << started.exchange(0)
                << ",\"completed\":" << completed.exchange(0)
                << ",\"aborted\":" << aborted.exchange(0)
                << ",\"submitted_frames\":" << submitted_frames.exchange(0)
                << ",\"submission_span\":";
            submission_span.json(out);
            out << '}';
        }
    } batch;
    DurationCounter read, write, read_lock, write_lock, pacing, sender;
    DurationCounter sender_wake_lateness, target_age_at_send, send_attempt_interval;
    SendExecutionTiming send_execution;
    std::atomic<uint64_t> rx_frames{0}, tx_attempts{0}, tx_failures{0};
    std::atomic<uint64_t> rx_budget_hits{0}, sender_busy_skips{0}, sender_faults{0};

    std::string snapshot(double seconds) {
        std::ostringstream out;
        out << "{\"schema_version\":\"hardware-timing.v1\",\"source\":\"odrive_hardware\","
            << "\"window_s\":" << seconds << ",\"read\":";
        read.json(out);
        out << ",\"write\":"; write.json(out);
        out << ",\"read_lock\":"; read_lock.json(out);
        out << ",\"write_lock\":"; write_lock.json(out);
        out << ",\"pacing\":"; pacing.json(out);
        out << ",\"sender\":"; sender.json(out);
        out << ",\"send_execution\":"; send_execution.json(out);
        out << ",\"sender_wake_lateness\":"; sender_wake_lateness.json(out);
        out << ",\"target_age_at_send\":"; target_age_at_send.json(out);
        out << ",\"send_attempt_interval\":"; send_attempt_interval.json(out);
        out << ",\"batch\":"; batch.json(out);
        out << ",\"rx_frames\":" << rx_frames.exchange(0)
            << ",\"tx_attempts\":" << tx_attempts.exchange(0)
            << ",\"tx_failures\":" << tx_failures.exchange(0)
            << ",\"rx_budget_hits\":" << rx_budget_hits.exchange(0)
            << ",\"sender_busy_skips\":" << sender_busy_skips.exchange(0)
            << ",\"sender_faults\":" << sender_faults.exchange(0) << '}';
        return out.str();
    }
};
}  // namespace odrive_ros2_control

#pragma once
#include "hardware_timing.hpp"
#include <cstdlib>
#include <limits>

namespace odrive_ros2_control {
// Updated under the existing CAN mutex; exporter touches only atomic counters.
// ROS header age is not encoder age. These metrics start at kernel receipt.
struct EncoderTiming {
    DurationCounter queue_delay, age_at_read;
    std::atomic<uint64_t> frames{0}, missing_stamp{0}, invalid_clock{0};
    std::atomic<uint64_t> never_received_reads{0}, unavailable_age_reads{0};
    bool seen = false, valid = false, have_offset = false;
    int64_t offset = 0, decoded_at = 0, queued_ns = 0;
    static constexpr int64_t clock_step_ns = 5000000;

    void reset_receipt() { seen = valid = have_offset = false; }

    // Linux steady/monotonic clock, same host/boot only. NaN means unavailable.
    double receipt_monotonic_seconds() const {
        return seen && valid && decoded_at >= queued_ns
            ? static_cast<double>(decoded_at - queued_ns) / 1e9
            : std::numeric_limits<double>::quiet_NaN();
    }

    void receive(int64_t kernel, int64_t wall, int64_t steady) {
        frames.fetch_add(1, std::memory_order_relaxed);
        seen = true;
        valid = false;
        const auto next_offset = wall - steady;
        const bool step = have_offset && std::llabs(next_offset - offset) > clock_step_ns;
        offset = next_offset;
        have_offset = true;
        if (kernel <= 0) { ++missing_stamp; return; }
        if (step || wall < kernel) { ++invalid_clock; return; }
        queued_ns = wall - kernel;
        decoded_at = steady;
        valid = true;
        queue_delay.add(queued_ns);
    }

    void sample(int64_t wall, int64_t steady) {
        if (!seen) { ++never_received_reads; return; }
        if (valid && (steady < decoded_at ||
            std::llabs((wall - steady) - offset) > clock_step_ns)) {
            valid = false;
            ++invalid_clock;
        }
        if (!valid) { ++unavailable_age_reads; return; }
        age_at_read.add(queued_ns + steady - decoded_at);
    }

    void json(std::ostream& out, uint32_t node_id) {
        out << "{\"node_id\":" << node_id << ",\"queue_delay\":";
        queue_delay.json(out);
        out << ",\"age_at_read\":"; age_at_read.json(out);
        out << ",\"frames\":" << frames.exchange(0)
            << ",\"missing_stamp\":" << missing_stamp.exchange(0)
            << ",\"invalid_clock\":" << invalid_clock.exchange(0)
            << ",\"never_received_reads\":" << never_received_reads.exchange(0)
            << ",\"unavailable_age_reads\":" << unavailable_age_reads.exchange(0) << '}';
    }
};
}

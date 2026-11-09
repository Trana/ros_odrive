#include "hardware_timing.hpp"

#include <iostream>
#include <stdexcept>

using namespace odrive_ros2_control;
void check(bool value) {
    if (!value)
        throw std::runtime_error("hardware timing assertion failed");
}
int main() {
    HardwareTiming timing;
    timing.read.add(1000000);
    timing.read.add(3000000);
    check(timing.read.count == 2);
    check(timing.read.sum_ns == 4000000);
    check(timing.read.max_ns == 3000000);
    timing.tx_attempts = 12;
    timing.tx_failures = 2;
    timing.sender_wake_lateness.add(200000);
    timing.target_age_at_send.add(1000000);
    timing.send_attempt_interval.add(1700000);
    const auto result = timing.snapshot(1.0);
    check(result.find("\"mean_ms\":2") != std::string::npos);
    check(result.find("\"max_ms\":3") != std::string::npos);
    check(result.find("\"tx_failures\":2") != std::string::npos);
    check(timing.read.count == 0 && timing.read.sum_ns == 0 && timing.read.max_ns == 0);
    check(timing.tx_attempts == 0 && timing.tx_failures == 0);
    check(result.find("\"sender_wake_lateness\":{\"count\":1") != std::string::npos);
    check(result.find("\"target_age_at_send\":{\"count\":1") != std::string::npos);
    check(result.find("\"send_attempt_interval\":{\"count\":1") != std::string::npos);
    check(timing.sender_wake_lateness.count == 0);
    check(timing.target_age_at_send.count == 0 && timing.send_attempt_interval.count == 0);
    { TimedScope disabled(nullptr); }
    check(timing.write.count == 0);
    { TimedScope enabled(&timing.write); }
    check(timing.write.count == 1);
    auto& execution = timing.send_execution;
    execution.add(1000000, 200000, true);
    execution.add(10000000, 1000000, true);
    execution.add(2000000, 2000000, true); // Inclusive slow threshold.
    execution.add(1000000, 0, false);
    execution.add(1000000, 1000001, true); // Inconsistent clocks are not clamped.
    check(execution.elapsed.count == 5 && execution.cpu.count == 3);
    check(execution.off_cpu.sum_ns == 9800000);
    check(execution.slow_elapsed.count == 2 && execution.slow_cpu.sum_ns == 3000000);
    check(execution.slow_off_cpu.sum_ns == 9000000 && execution.invalid_cpu_samples == 2);
    const auto paired = timing.snapshot(1);
    check(paired.find("\"send_execution\":{") != std::string::npos);
    check(paired.find("\"invalid_cpu_samples\":2") != std::string::npos);
    check(execution.cpu.count == 0 && execution.off_cpu.sum_ns == 0);
    check(execution.slow_elapsed.count == 0 && execution.invalid_cpu_samples == 0);
    { SendExecutionScope disabled(nullptr); }
    check(execution.elapsed.count == 0);
    { SendExecutionScope enabled(&execution); }
    check(execution.elapsed.count == 1);
    check(execution.cpu.count + execution.invalid_cpu_samples == 1);
    std::cout << "Hardware timing scope and snapshot tests passed\n";
}

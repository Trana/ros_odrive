#include "paced_commands.hpp"

#include <iostream>
#include <stdexcept>
using namespace odrive_ros2_control;
using namespace std::chrono_literals;
void check(bool ok) {
    if (!ok)
        throw std::runtime_error("batch assertion failed");
}
int main() {
    PacedCommands m;
    HardwareTiming timing;
    m.set_batch_timing(&timing.batch);
    m.configure(12, 50, 250, false, false, true);
    auto now = PacedCommands::Clock::time_point{};
    auto stage = [&](int generation) {
        for (int i = 0; i < 12; ++i) {
            can_frame f{};
            f.can_id = i;
            f.data[0] = generation;
            m.set(i, f);
        }
        m.commit(now);
    };
    stage(1);
    check(!m.due(now));
    m.activate(now);
    check(!m.due(now));
    stage(1);
    const auto start = now;
    for (int i = 0; i < 12; ++i) {
        auto f = m.due(now);
        check(f && f->frame.can_id == static_cast<unsigned>(i) && f->frame.data[0] == 1);
        stage(2); // Updates must not alter the in-flight snapshot.
        check(m.target_age(now) == now - start);
        now += 155us;
        m.finish_submission(now, true);
        if (i < 11)
            check(m.wake_after(now) == now);
    }
    check(!m.due(now));
    check(timing.batch.started == 1 && timing.batch.completed == 1);
    check(timing.batch.submitted_frames == 12 && timing.batch.aborted == 0);
    check(timing.batch.submission_span.max_ns == 1860000);
    check(m.wake_after(now) == start + 20ms);
    now = start + 20ms;
    check(m.due(now)->frame.data[0] == 2);
    // Mode switch/stop cancels the remaining old snapshot.
    m.discard();
    check(!m.due(now));
    check(timing.batch.aborted == 1);
    stage(3);
    check(m.due(now)->frame.data[0] == 3);
    // Fresh writes do not rescue an excessively delayed partial batch.
    now += 20ms;
    stage(4);
    check(!m.due(now) && m.faulted());
    check(m.fault_reason() == PacedCommands::FaultReason::batch_deadline);
    check(timing.batch.aborted == 2);
    m.activate(now);
    check(!m.due(now));
    stage(5);
    check(m.due(now).has_value());
    m.fault(PacedCommands::FaultReason::send_failure);
    check(!m.due(now));
    m.activate(now);
    stage(6);
    now += 250ms;
    check(!m.due(now) && m.faulted());
    check(m.fault_reason() == PacedCommands::FaultReason::producer_stale);
    // Missing interfaces are skipped, no allocation or infinite retry.
    m.activate(now);
    m.set(0, std::nullopt);
    m.commit(now);
    for (int i = 0; i < 12; ++i) {
        check(!m.due(now));
        m.finish_submission(now, false);
    }
    check(m.wake_after(now) == now + 20ms);
    check(timing.batch.completed == 2); // Empty interface round, zero new submissions.
    check(timing.batch.submitted_frames == 12);
    const auto json = timing.snapshot(1);
    check(json.find("\"completed\":2") != std::string::npos);
    check(timing.batch.started == 0 && timing.batch.aborted == 0);
    check(timing.batch.submission_span.count == 0);
    // Last-frame failure must not appear as a completed batch.
    m.activate(now);
    stage(7);
    for (int i = 0; i < 11; ++i) {
        check(m.due(now).has_value());
        m.finish_submission(now, true);
    }
    check(m.due(now).has_value());
    m.fault(PacedCommands::FaultReason::send_failure);
    check(timing.batch.completed == 0 && timing.batch.aborted == 1);
    check(timing.batch.submitted_frames == 11);
    // Spaced batches retain one generation and measure gaps after completion,
    // including when tracing is disabled (no timing object attached).
    PacedCommands spaced;
    spaced.configure(12, 50, 250, false, false, true, 500);
    now = PacedCommands::Clock::time_point{};
    spaced.activate(now);
    for (int i = 0; i < 12; ++i) {
        can_frame f{};
        f.can_id = i;
        f.data[0] = 10;
        spaced.set(i, f);
    }
    spaced.commit(now);
    const auto spaced_start = now;
    for (int i = 0; i < 12; ++i) {
        auto f = spaced.due(now);
        check(f && f->frame.can_id == static_cast<unsigned>(i) && f->frame.data[0] == 10);
        can_frame replacement{};
        replacement.data[0] = 20;
        spaced.set(11, replacement);
        spaced.commit(now);
        now += 150us; // Simulated socket send work.
        spaced.finish_submission(now, true);
        if (i < 11) {
            check(spaced.wake_after(now) == now + 500us);
            check(!spaced.due(now + 499us));
            now += 500us;
        }
    }
    check(spaced.wake_after(now) == spaced_start + 20ms);
    now = spaced_start + 20ms;
    check(spaced.due(now).has_value());
    spaced.finish_submission(now, true);
    spaced.inhibit();
    check(!spaced.due(now + 500us));
    spaced.activate(now);
    spaced.set(0, can_frame{});
    spaced.commit(now);
    check(spaced.due(now).has_value());
    spaced.finish_submission(now, true);
    // Late wake must fault rather than squeeze missed sends into a burst.
    now += 20ms;
    spaced.commit(now);
    check(!spaced.due(now) && spaced.faulted());
    check(spaced.fault_reason() == PacedCommands::FaultReason::batch_deadline);
    check(std::string(spaced.fault_reason_name()) == "batch_deadline");
    spaced.fault(PacedCommands::FaultReason::send_failure);
    spaced.inhibit();
    check(spaced.fault_reason() == PacedCommands::FaultReason::batch_deadline);
    spaced.clear_fault_after_idle();
    check(!spaced.faulted() && std::string(spaced.fault_reason_name()) == "none");
    spaced.fault(PacedCommands::FaultReason::non_finite_target);
    check(std::string(spaced.fault_reason_name()) == "non_finite_target");
    spaced.activate(now);
    check(!spaced.faulted() && spaced.fault_reason() == PacedCommands::FaultReason::none);
    spaced.fault(PacedCommands::FaultReason::send_failure);
    check(std::string(spaced.fault_reason_name()) == "send_failure");
    // Slow round, shorter stale limit: isolate snapshot age from producer age.
    spaced.configure(12, 1, 20, false, false, true, 500);
    spaced.activate(now);
    spaced.set(0, can_frame{});
    spaced.commit(now);
    check(spaced.due(now).has_value());
    spaced.finish_submission(now, true);
    now += 20ms;
    spaced.commit(now);
    check(!spaced.due(now) && spaced.faulted());
    check(std::string(spaced.fault_reason_name()) == "snapshot_stale");
    spaced.activate(now);
    spaced.set(0, can_frame{});
    spaced.commit(now);
    now += 20ms;
    check(!spaced.due(now) && spaced.faulted());
    check(std::string(spaced.fault_reason_name()) == "producer_stale");
    for (double gap : {-1.0, 1001.0, std::nan(""), HUGE_VAL}) {
        bool rejected = false;
        try {
            spaced.configure(12, 50, 250, false, false, true, gap);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        check(rejected);
    }
    for (bool batch : {false, true}) {
        bool rejected = false;
        try {
            spaced.configure(12, 200, 250, false, false, batch, 500);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        check(rejected); // Non-batch setting or impossible inter-slot budget.
    }
    std::cout << "Coherent/spaced snapshot, gap, deadline, stop and stale checks passed\n";
}

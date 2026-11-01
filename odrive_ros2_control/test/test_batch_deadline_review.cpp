#include "paced_commands.hpp"
#include <iostream>
#include <stdexcept>

using namespace odrive_ros2_control;
using namespace std::chrono_literals;
using Time = PacedCommands::Clock::time_point;

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

void stage(PacedCommands& mailbox, Time now, unsigned char generation = 1) {
    for (unsigned i = 0; i < 12; ++i) {
        can_frame frame{};
        frame.can_id = i;
        frame.data[0] = generation;
        mailbox.set(i, frame);
    }
    mailbox.commit(now);
}

// Decision table ONLY: not an alternative sender implementation. Synthetic
// bounds deliberately differ from production; none are approved robot limits.
enum class Decision { send, stop, producer_stale, snapshot_stale, spread };
Decision independent_limits(bool enabled, std::chrono::milliseconds producer_age,
                            std::chrono::milliseconds snapshot_age,
                            std::chrono::milliseconds elapsed) {
    if (!enabled) return Decision::stop;
    if (producer_age >= 60ms) return Decision::producer_stale;
    if (snapshot_age >= 60ms) return Decision::snapshot_stale;
    if (elapsed >= 30ms) return Decision::spread;
    return Decision::send;
}

int main() {
    const Time origin{};
    PacedCommands mailbox;
    HardwareTiming timing;
    mailbox.set_batch_timing(&timing.batch);
    mailbox.configure(12, 50, 250, false, false, true, 500);

    // Late first wake is not a partial-batch deadline violation. No old rounds
    // are replayed; the snapshot and period anchor at the actual start.
    mailbox.activate(origin);
    stage(mailbox, origin);
    stage(mailbox, origin + 99ms, 2);
    auto frame = mailbox.due(origin + 100ms);
    check(frame && frame->frame.data[0] == 2, "late start must select latest snapshot");
    mailbox.finish_submission(origin + 100ms, true);
    stage(mailbox, origin + 119ms, 3);
    check(mailbox.due(origin + 119ms)->frame.data[0] == 2, "snapshot must stay frozen");
    mailbox.finish_submission(origin + 119ms, true);
    check(mailbox.target_age(origin + 119ms) == 20ms, "new writes must not reset snapshot age");
    check(!mailbox.due(origin + 120ms), "exact period boundary must reject partial batch");
    check(mailbox.fault_reason() == PacedCommands::FaultReason::batch_deadline,
          "expected batch deadline despite fresh producer");

    // Characterize, do not endorse: the last send may finish after the cutoff.
    mailbox.activate(origin);
    stage(mailbox, origin);
    for (int i = 0; i < 11; ++i) {
        const auto now = origin + i * 1ms;
        check(mailbox.due(now).has_value(), "expected initial frame");
        mailbox.finish_submission(now, true);
    }
    check(mailbox.due(origin + 19999us).has_value(), "last send allowed just before cutoff");
    const auto completed_before = timing.batch.completed.load();
    mailbox.finish_submission(origin + 23ms, true);
    check(!mailbox.faulted(), "current finish has no post-send deadline check");
    check(timing.batch.completed == completed_before + 1, "late completion is counted today");
    check(mailbox.wake_after(origin + 23ms) == origin + 43ms,
          "overrun must not trigger immediate catch-up batch");
    mailbox.inhibit();
    stage(mailbox, origin + 24ms);
    check(!mailbox.due(origin + 50ms), "stop must prevent resumed sends");

    // Proposed separation: missing nominal 20 ms need not itself mean stale.
    check(independent_limits(true, 1ms, 25ms, 21ms) == Decision::send, "independent cadence");
    check(independent_limits(true, 1ms, 29ms, 29ms) == Decision::send, "below synthetic spread bound");
    check(independent_limits(true, 1ms, 30ms, 30ms) == Decision::spread, "exact spread boundary");
    check(independent_limits(true, 1ms, 60ms, 10ms) == Decision::snapshot_stale,
          "fresh producer cannot rescue old snapshot");
    check(independent_limits(true, 60ms, 60ms, 10ms) == Decision::producer_stale,
          "producer boundary");
    check(independent_limits(false, 0ms, 0ms, 0ms) == Decision::stop, "stop overrides freshness");

    // Counterexample to blindly dropping partial rounds and restarting at axis
    // zero: repeated stalls after six sends permanently starve axes 6..11.
    unsigned sends[12]{};
    for (int generation = 0; generation < 100; ++generation)
        for (unsigned axis = 0; axis < 6; ++axis) ++sends[axis];
    check(sends[0] == 100 && sends[11] == 0, "drop/restart counterexample");
    std::cout << "Deadline characterization and test-only decision cases passed\n";
}

#include "paced_commands.hpp"
#include <iostream>
#include <stdexcept>

using namespace odrive_ros2_control;
using namespace std::chrono_literals;
void check(bool ok) { if (!ok) throw std::runtime_error("paced command assertion failed"); }
int main() {
  for (int mode : {0, 1, 2}) {
    const bool phase = mode != 0;
    PacedCommands mailbox;
    mailbox.configure(12, 50, 250, mode == 1, mode == 2);
    auto now = PacedCommands::Clock::time_point{};
    can_frame frame{}; frame.can_id = 42;
    mailbox.set(0, frame); mailbox.commit(now);
    check(!mailbox.due(now)); // Disabled startup cannot send.
    mailbox.activate(now);
    check(!mailbox.due(now)); // Activation discards pre-activation targets.
    for (size_t i = 0; i < 12; ++i) { frame.can_id = i; mailbox.set(i, frame); }
    mailbox.commit(now);
    check(mailbox.due(now)->frame.can_id == 0);
    check(mailbox.target_age(now + 200us) == 200us);
    check(mailbox.wake_after(now + 200us) == now + mailbox.slot());
    check(mailbox.wake_after(now + mailbox.slot()) == now + 2 * mailbox.slot());
    // Send cost is paid within the interval, not added to it on each axis.
    auto deadline = mailbox.wake_after(now + 200us);
    check(!mailbox.due(deadline - 1ns));
    check(!mailbox.due(now)); // No immediate second frame.
    // Hundreds of producer updates consume no additional slots/history.
    for (int i = 0; i < 1000; ++i) { frame.can_id = 100 + i; mailbox.set(1, frame); mailbox.commit(now); }
    now += mailbox.slot();
    check(mailbox.due(now)->frame.can_id == 1099);
    check(mailbox.target_age(now) == mailbox.slot());
    now += 100ms; // Late sender skips missed times, no catch-up burst.
    check(mailbox.due(now)->frame.can_id == 2);
    check(!mailbox.due(now));
    now += 250ms;
    check(!mailbox.due(now) && mailbox.faulted());
    mailbox.commit(now); // Fresh submissions alone cannot clear a fault.
    check(!mailbox.due(now));
    mailbox.inhibit();
    mailbox.clear_fault_after_idle();
    mailbox.activate(now);
    check(!mailbox.due(now));
    mailbox.set(0, frame); mailbox.commit(now);
    mailbox.discard(); // Mode change invalidates queued targets.
    check(!mailbox.due(now));
    mailbox.set(0, frame); mailbox.commit(now);
    mailbox.inhibit(); // Stop invalidates already staged targets.
    check(!mailbox.due(now + 1s));
    for (double rate : {0., 201., std::nan("")}) {
        bool rejected = false;
        try { mailbox.configure(12, rate, 250, phase); } catch (const std::invalid_argument&) { rejected = true; }
        check(rejected);
    }
    mailbox.configure(12, 50, 250, mode == 1, mode == 2);
    now = PacedCommands::Clock::time_point{};
    mailbox.activate(now);
    for (size_t i = 0; i < 12; ++i) mailbox.set(i, frame);
    for (int i = 0; i < 1200; ++i) {
        mailbox.commit(now);
        check(mailbox.due(now).has_value());
        now = mailbox.wake_after(now + 200us);
    }
    check(now.time_since_epoch() == 1200 * mailbox.slot());
    // Overrun waits a complete slot instead of sending a catch-up frame.
    mailbox.commit(now);
    check(mailbox.due(now).has_value());
    const auto completed = now + 10ms;
    check(mailbox.wake_after(completed) >= completed + mailbox.slot());
    if (!phase) check(mailbox.wake_after(completed) == completed + mailbox.slot());
  }
    std::cout << "Pacing, replacement, stall, stale, mode-change and stop tests passed\n";
}

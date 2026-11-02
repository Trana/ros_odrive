// Deterministic clock experiment only; no ROS, sockets or motor commands.
// Exercises both modes of the real mailbox; default pacing remains relative.
#include "paced_commands.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>

using namespace odrive_ros2_control;
using namespace std::chrono_literals;
using Clock = PacedCommands::Clock;

struct Result { double hz, min_ms, max_ms; };

Result experiment(int mode, bool jitter) {
    PacedCommands mailbox;
    mailbox.configure(12, 50, 250, mode == 1, mode == 2);
    auto now = Clock::time_point{};
    auto deadline = now;
    auto previous = now;
    auto first = now;
    auto minimum = 1s + Clock::duration::zero();
    auto maximum = Clock::duration::zero();
    mailbox.activate(now);
    can_frame frame{};
    for (size_t axis = 0; axis < 12; ++axis) mailbox.set(axis, frame);
    constexpr int count = 12000;
    for (int i = 0; i < count; ++i) {
        // Synthetic wake delay; not a replay of the measured distribution.
        const auto late = jitter && i % 17 == 0 ? 1200us : 152us;
        now = deadline + late;
        if (!i) first = now;
        if (i) {
            minimum = std::min(minimum, now - previous);
            maximum = std::max(maximum, now - previous);
        }
        previous = now;
        mailbox.commit(now);
        if (!mailbox.due(now)) throw std::runtime_error("missing production slot");
        deadline = mailbox.wake_after(now + 155us);
    }
    return {(count - 1) / std::chrono::duration<double>(now - first).count() / 12,
            std::chrono::duration<double, std::milli>(minimum).count(),
            std::chrono::duration<double, std::milli>(maximum).count()};
}

int main() {
    for (bool jitter : {false, true}) {
        for (int mode : {0, 1, 2}) {
            const auto r = experiment(mode, jitter);
            std::cout << (mode == 2 ? "bounded_phase" : mode == 1 ? "phase_90pct" : "production")
                      << " jitter=" << jitter << " hz_per_motor=" << r.hz
                      << " min_gap_ms=" << r.min_ms << " max_gap_ms=" << r.max_ms << '\n';
            if (r.min_ms < 1.499 || r.hz > 50.01)
                throw std::runtime_error("spacing/rate bound violated");
            if (!jitter && mode && r.hz < 49.99)
                throw std::runtime_error("phase drift under constant lateness");
            if (!jitter && !mode && (r.hz < 45.8 || r.hz > 45.9))
                throw std::runtime_error("production drift model changed");
        }
    }
    // Adversarial deterministic sequence: wake jitter, long stalls, send work
    // overruns and mutex-busy retries. All state checks use the real mailbox.
    for (int mode : {0, 1, 2}) {
        PacedCommands m;
        m.configure(12, 50, 250, mode == 1, mode == 2);
        auto now = Clock::time_point{};
        auto previous = now;
        auto deadline = now;
        m.activate(now);
        can_frame f{};
        for (size_t axis = 0; axis < 12; ++axis) { f.can_id = axis; m.set(axis, f); }
        uint32_t random = 12345;
        size_t sent = 0;
        for (int i = 0; i < 100000; ++i) {
            random = random * 1664525u + 1013904223u;
            now = deadline + std::chrono::nanoseconds(random % 2000000);
            if (i % 101 == 0) now += 50ms;
            if (i % 13 == 0) { // Mirrors busy transport path: no due() call.
                deadline = now + m.slot();
                continue;
            }
            m.commit(now);
            const auto frame = m.due(now);
            if (!frame || frame->frame.can_id != sent % 12)
                throw std::runtime_error("lost round-robin order");
            if (sent && now - previous < (mode ? m.slot() * 9 / 10 : m.slot()))
                throw std::runtime_error("compressed frame spacing");
            if (m.due(now)) throw std::runtime_error("catch-up burst");
            previous = now;
            ++sent;
            const auto completed = now + std::chrono::nanoseconds(random % 4000000);
            deadline = m.wake_after(completed);
            if (deadline <= completed) throw std::runtime_error("busy-spin deadline");
        }
        // Stalled producer faults before sending; fresh commit cannot bypass it.
        now += 250ms;
        if (m.due(now) || !m.faulted()) throw std::runtime_error("stale target sent");
        m.commit(now);
        if (m.due(now + 1s)) throw std::runtime_error("fault bypassed");
        m.activate(now);
        if (m.due(now)) throw std::runtime_error("old target survived activation");
        std::cout << "adversarial mode=" << mode << " opportunities=100000 passed\n";
    }
}

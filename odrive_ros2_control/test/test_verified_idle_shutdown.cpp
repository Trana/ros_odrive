#include "verified_idle_shutdown.hpp"
#include <iostream>
#include <stdexcept>
#include <string>

using odrive_ros2_control::VerifiedIdleShutdown;
using namespace std::chrono_literals;

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

can_frame heartbeat(uint32_t id, uint8_t state) {
    can_frame frame{};
    frame.can_id = (id << 5) | Heartbeat_msg_t::cmd_id;
    frame.can_dlc = Heartbeat_msg_t::msg_length;
    Heartbeat_msg_t msg;
    msg.Axis_State = state;
    msg.encode_buf(frame.data);
    return frame;
}

struct Fixture {
    std::chrono::steady_clock::time_point time{};
    template <typename Send, typename Poll>
    bool run(VerifiedIdleShutdown& shutdown, Send send, Poll poll) {
        return shutdown.run(send, poll, [this] { return time; },
            [this](auto duration) { time += duration; });
    }
};

int main() {
    try {
        const std::vector<uint32_t> ids{11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43};
        {
            Fixture f;
            VerifiedIdleShutdown shutdown(ids);
            int sends = 0;
            check(f.run(shutdown, [&](auto) { ++sends; return true; }, [&] {
                // The normal one-second heartbeat period fits the deadline.
                if (f.time.time_since_epoch() >= 1000ms) {
                    for (auto id : ids) shutdown.observe(heartbeat(id, AXIS_STATE_IDLE));
                }
            }), "all twelve fresh idle heartbeats must succeed");
            check(sends > 12, "missing acknowledgements must trigger retries");
        }
        {
            Fixture f;
            VerifiedIdleShutdown shutdown({11, 43});
            int attempts11 = 0, attempts43 = 0;
            check(f.run(shutdown, [&](auto id) {
                if (id == 11) { ++attempts11; return true; }
                return ++attempts43 >= 3;
            }, [&] {
                shutdown.observe(heartbeat(11, AXIS_STATE_IDLE));
                // A heartbeat cannot confirm a node whose sends all failed.
                shutdown.observe(heartbeat(43, AXIS_STATE_IDLE));
            }), "transient send failures must recover");
            check(attempts11 == 1 && attempts43 == 3, "retry only unresolved drives");
            check(shutdown.drives()[1].send_failures == 2, "retain send failure diagnostics");
        }
        {
            Fixture f;
            VerifiedIdleShutdown shutdown({11, 43});
            shutdown.observe(heartbeat(43, AXIS_STATE_IDLE));
            check(!f.run(shutdown, [](auto) { return true; }, [&] {
                shutdown.observe(heartbeat(11, AXIS_STATE_IDLE));
            }), "old idle heartbeat and silent node must not succeed");
            check(f.time.time_since_epoch() == 1500ms, "timeout must be bounded");
            check(shutdown.drives()[0].idle_confirmed && !shutdown.drives()[1].idle_confirmed,
                "identify the unconfirmed drive");
        }
        {
            Fixture f;
            VerifiedIdleShutdown shutdown({43});
            check(!f.run(shutdown, [](auto) { return false; }, [&] {
                shutdown.observe(heartbeat(43, AXIS_STATE_IDLE));
            }), "permanent send failure must never be reported as success");
            check(shutdown.drives()[0].send_failures == 15, "retries must be paced and bounded");
        }
        {
            Fixture f;
            VerifiedIdleShutdown shutdown({43});
            check(!f.run(shutdown, [](auto) { return true; }, [&] {
                shutdown.observe(heartbeat(43, AXIS_STATE_CLOSED_LOOP_CONTROL));
                shutdown.observe(heartbeat(42, AXIS_STATE_IDLE));
                auto short_frame = heartbeat(43, AXIS_STATE_IDLE);
                short_frame.can_dlc = 4;
                shutdown.observe(short_frame);
                for (auto flag : {CAN_RTR_FLAG, CAN_ERR_FLAG, CAN_EFF_FLAG}) {
                    auto frame = heartbeat(43, AXIS_STATE_IDLE);
                    frame.can_id |= flag;
                    shutdown.observe(frame);
                }
            }), "active, wrong-node, short and flagged frames must not confirm shutdown");
        }
        {
            Fixture f;
            VerifiedIdleShutdown shutdown({11, 43});
            check(!f.run(shutdown, [](auto) { return true; }, [&] {
                shutdown.observe(heartbeat(11, AXIS_STATE_IDLE));
                shutdown.observe(heartbeat(11, AXIS_STATE_CLOSED_LOOP_CONTROL));
                shutdown.observe(heartbeat(43, AXIS_STATE_IDLE));
            }), "later active heartbeat must invalidate idle confirmation");
        }
        std::cout << "Verified idle shutdown: six fault-injection scenarios passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

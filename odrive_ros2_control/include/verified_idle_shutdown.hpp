#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <vector>

#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "odrive_enums.h"
#include <linux/can.h>

namespace odrive_ros2_control {

// Each attempt starts without acknowledgements. The caller drains queued CAN
// frames before starting, then pumps reception while normal writes are inhibited.
class VerifiedIdleShutdown {
public:
    struct Drive {
        uint32_t node_id;
        bool request_sent = false;
        bool idle_confirmed = false;
        std::size_t send_failures = 0;
    };

    explicit VerifiedIdleShutdown(const std::vector<uint32_t>& node_ids) {
        for (auto id : node_ids) drives_.push_back(Drive{id});
    }

    void observe(const can_frame& frame) {
        if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0 ||
            (frame.can_id & 0x1f) != Heartbeat_msg_t::cmd_id ||
            frame.can_dlc < Heartbeat_msg_t::msg_length) return;
        for (auto& drive : drives_) {
            if (drive.node_id == (frame.can_id >> 5) && drive.request_sent) {
                Heartbeat_msg_t heartbeat;
                heartbeat.decode_buf(frame.data);
                drive.idle_confirmed = heartbeat.Axis_State == AXIS_STATE_IDLE;
            }
        }
    }

    bool complete() const {
        return !drives_.empty() && std::all_of(drives_.begin(), drives_.end(),
            [](const Drive& drive) { return drive.idle_confirmed; });
    }

    const std::vector<Drive>& drives() const { return drives_; }

    // Callbacks permit deterministic fault-injection tests without hardware.
    template <typename Send, typename Poll, typename Now, typename Sleep>
    bool run(Send send_idle, Poll poll, Now now, Sleep sleep) {
        using namespace std::chrono_literals;
        const auto deadline = now() + 1500ms;
        auto next_retry = now();
        while (now() < deadline) {
            if (now() >= next_retry) {
                for (auto& drive : drives_) {
                    if (!drive.idle_confirmed) {
                        if (send_idle(drive.node_id)) drive.request_sent = true;
                        else ++drive.send_failures;
                    }
                }
                next_retry = now() + 100ms;
            }
            poll();
            if (complete()) return true;
            sleep(5ms);
        }
        return false;
    }

private:
    std::vector<Drive> drives_;
};

}  // namespace odrive_ros2_control

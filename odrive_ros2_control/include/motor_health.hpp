#pragma once
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"

#include <cmath>
#include <linux/can.h>

// Read-only CAN evidence. Re-publishing these values never renews receipt time.
struct MotorHealth {
    double node_id = NAN, axis_state = NAN, heartbeat_error = NAN;
    double active_errors = NAN, disarm_reason = NAN, procedure_result = NAN;
    double heartbeat_receipt = NAN, error_receipt = NAN;
    bool observe(const can_frame& frame, double receipt) {
        if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG))
            return false;
        if (!std::isfinite(receipt) || receipt <= 0)
            return false;
        const auto command = frame.can_id & 0x1f;
        if (command == Heartbeat_msg_t::cmd_id && frame.can_dlc == Heartbeat_msg_t::msg_length) {
            Heartbeat_msg_t msg;
            msg.decode_buf(frame.data);
            const bool changed = axis_state != msg.Axis_State || heartbeat_error != msg.Axis_Error;
            axis_state = msg.Axis_State;
            heartbeat_error = msg.Axis_Error;
            procedure_result = msg.Procedure_Result;
            heartbeat_receipt = receipt;
            return changed;
        }
        if (command == Get_Error_msg_t::cmd_id && frame.can_dlc == Get_Error_msg_t::msg_length) {
            Get_Error_msg_t msg;
            msg.decode_buf(frame.data);
            const bool changed = active_errors != msg.Active_Errors || disarm_reason != msg.Disarm_Reason;
            active_errors = msg.Active_Errors;
            disarm_reason = msg.Disarm_Reason;
            error_receipt = receipt;
            return changed;
        }
        return false;
    }
};

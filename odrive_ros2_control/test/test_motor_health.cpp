#include "motor_health.hpp"
#include <cassert>
int main() {
    MotorHealth h;
    assert(std::isnan(h.active_errors));
    can_frame f{}; f.can_id = (42<<5)|Heartbeat_msg_t::cmd_id; f.can_dlc=Heartbeat_msg_t::msg_length;
    Heartbeat_msg_t hb; hb.Axis_State=1; hb.Axis_Error=512; hb.encode_buf(f.data);
    assert(h.observe(f,10)); assert(h.axis_state==1 && h.heartbeat_error==512);
    assert(std::isnan(h.active_errors)); // Heartbeat is not Get_Error.
    f.can_id=(42<<5)|Get_Error_msg_t::cmd_id; f.can_dlc=8;
    Get_Error_msg_t e; e.Disarm_Reason=512; e.encode_buf(f.data);
    assert(h.observe(f,11)); assert(h.active_errors==0 && h.disarm_reason==512);
    assert(h.heartbeat_receipt==10 && h.error_receipt==11);
    assert(!h.observe(f,NAN)); assert(h.error_receipt==11);
    f.can_dlc=2; assert(!h.observe(f,12)); assert(h.error_receipt==11);
    f.can_dlc=8; f.can_id|=CAN_RTR_FLAG; assert(!h.observe(f,12));
}

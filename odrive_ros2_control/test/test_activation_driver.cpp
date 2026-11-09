// Production driver, fake CAN transport. This executable never opens a socket.
#include "../src/odrive_hardware_interface.cpp"

#include <deque>
#include <iostream>
#include <stdexcept>

namespace {
struct ReceivedFrame {
    can_frame frame;
    int64_t receipt;
};
std::deque<ReceivedFrame> incoming;
std::mutex sent_mutex;
std::vector<can_frame> sent;
int failed_prime_node = -1;
int failed_enable_node = -1;
void check(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}
double steady_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t wall_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}
void encoder(int node, float motor_position, int64_t age = 0) {
    can_frame frame{};
    frame.can_id = (node << 5) | Get_Encoder_Estimates_msg_t::cmd_id;
    frame.can_dlc = 8;
    Get_Encoder_Estimates_msg_t sample{};
    sample.Pos_Estimate = motor_position;
    sample.encode_buf(frame.data);
    incoming.push_back({frame, wall_ns() - age});
}
std::vector<can_frame> take_sent() {
    std::lock_guard lock(sent_mutex);
    auto result = sent;
    sent.clear();
    return result;
}
void check_hold(const can_frame& frame, float expected) {
    check((frame.can_id & 31) == Set_Input_Pos_msg_t::cmd_id, "expected position frame");
    Set_Input_Pos_msg_t msg{};
    msg.decode_buf(frame.data);
    check(std::abs(msg.Input_Pos - expected) < 1e-6, "wrong motor hold target");
    check(msg.Vel_FF == 0 && msg.Torque_FF == 0, "hold feedforward must be zero");
}
} // namespace

bool SocketCanIntf::init(const std::string&, EpollEventLoop*, FrameProcessor processor) {
    frame_processor_ = std::move(processor);
    return true;
}
void SocketCanIntf::deinit() {}
bool SocketCanIntf::read_nonblocking() {
    if (incoming.empty())
        return false;
    auto sample = incoming.front();
    incoming.pop_front();
    last_receive_timestamp_ns_ = sample.receipt;
    frame_processor_(sample.frame);
    return true;
}
bool SocketCanIntf::send_can_frame(const can_frame& frame) {
    {
        std::lock_guard lock(sent_mutex);
        sent.push_back(frame);
    }
    const auto node = frame.can_id >> 5;
    const auto command = frame.can_id & 31;
    if (command == Set_Input_Pos_msg_t::cmd_id && static_cast<int>(node) == failed_prime_node)
        return false;
    if (command == Set_Axis_State_msg_t::cmd_id && frame.data[0] == AXIS_STATE_CLOSED_LOOP_CONTROL
        && static_cast<int>(node) == failed_enable_node)
        return false;
    if (command == Set_Axis_State_msg_t::cmd_id && frame.data[0] == AXIS_STATE_IDLE) {
        can_frame heartbeat{};
        heartbeat.can_id = (node << 5) | Heartbeat_msg_t::cmd_id;
        heartbeat.can_dlc = 8;
        heartbeat.data[4] = AXIS_STATE_IDLE;
        incoming.push_back({heartbeat, wall_ns()});
    }
    return true;
}

int main() {
    try {
        ODriveHardwareInterface driver;
        hardware_interface::HardwareInfo info;
        info.name = "activation_test";
        info.type = "system";
        info.hardware_parameters["can"] = "fake_only";
        info.hardware_parameters["require_command_receipt"] = "true";
        info.hardware_parameters["can_tx_pacing"] = "coherent_batch";
        for (int node : {11, 12}) {
            hardware_interface::ComponentInfo joint;
            joint.name = "joint_" + std::to_string(node);
            joint.type = "joint";
            joint.parameters["node_id"] = std::to_string(node);
            info.joints.push_back(joint);
        }
        const rclcpp_lifecycle::State state;
        const rclcpp::Time time(0);
        const rclcpp::Duration period(0, 5000000);
        check(driver.on_init(info) == CallbackReturn::SUCCESS, "init");
        check(driver.on_configure(state) == CallbackReturn::SUCCESS, "configure");
        driver.perform_command_mode_switch({"joint_11/position", "joint_12/position"}, {});
        auto interfaces = driver.export_command_interfaces();
        auto command = [&](double target, double receipt) {
            for (auto& interface : interfaces) {
                if (interface.get_interface_name() == "position")
                    (void)interface.set_value(target);
                if (interface.get_interface_name() == "command_receipt_monotonic_s")
                    (void)interface.set_value(receipt);
            }
        };
        take_sent();
        check(driver.on_activate(state) == CallbackReturn::FAILURE, "missing feedback must reject");
        check(take_sent().empty(), "missing feedback must not send motor requests");
        encoder(11, .25f, 200000000);
        encoder(12, -.4f);
        driver.read(time, period);
        check(driver.on_activate(state) == CallbackReturn::FAILURE, "queued stale feedback must reject");
        check(take_sent().empty(), "stale feedback must not send motor requests");
        encoder(11, NAN);
        encoder(12, -.4f);
        driver.read(time, period);
        check(driver.on_activate(state) == CallbackReturn::FAILURE, "nonfinite position must reject");
        check(take_sent().empty(), "nonfinite feedback must not send motor requests");
        encoder(11, .25f);
        encoder(12, -.4f);
        driver.read(time, period);
        failed_prime_node = 12;
        check(driver.on_activate(state) == CallbackReturn::FAILURE, "failed prime must reject");
        auto frames = take_sent();
        check(frames.size() == 2, "failed prime must send no closed-loop requests");
        check_hold(frames[0], .25f);
        check_hold(frames[1], -.4f);
        failed_prime_node = -1;
        failed_enable_node = 12;
        check(driver.on_activate(state) == CallbackReturn::FAILURE, "failed enable must not report success");
        frames = take_sent();
        check(frames.size() == 4, "failed enable must not add automatic idle requests");
        check_hold(frames[0], .25f);
        check_hold(frames[1], -.4f);
        for (size_t i = 2; i < 4; ++i)
            check(frames[i].data[0] == AXIS_STATE_CLOSED_LOOP_CONTROL, "retain primed support on failed enable");
        failed_enable_node = -1;
        const auto old_receipt = steady_s();
        command(0, old_receipt);
        check(driver.on_activate(state) == CallbackReturn::SUCCESS, "fresh feedback activation");
        frames = take_sent();
        check(frames.size() == 4, "two primes then two enables");
        check_hold(frames[0], .25f);
        check_hold(frames[1], -.4f);
        for (size_t i = 2; i < 4; ++i)
            check(
                (frames[i].can_id & 31) == Set_Axis_State_msg_t::cmd_id
                    && frames[i].data[0] == AXIS_STATE_CLOSED_LOOP_CONTROL,
                "all primes must precede enables"
            );
        auto check_written = [&](float first, float second) {
            driver.write(time, period);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            auto written = take_sent();
            check(!written.empty(), "expected paced commands");
            for (const auto& frame : written)
                check_hold(frame, (frame.can_id >> 5) == 11 ? first : second);
        };
        command(0, old_receipt);
        check_written(.25f, -.4f);
        command(0, NAN);
        check_written(.25f, -.4f);
        command(.3, steady_s());
        const float motor = -.3 * 9 / (2 * M_PI);
        check_written(motor, motor);
        check(driver.on_deactivate(state) == CallbackReturn::SUCCESS, "stop");
        encoder(11, .7f);
        encoder(12, -.8f);
        driver.read(time, period);
        take_sent();
        check(driver.on_activate(state) == CallbackReturn::SUCCESS, "reactivate at changed pose");
        frames = take_sent();
        check_hold(frames[0], .7f);
        check_hold(frames[1], -.8f);
        command(0, old_receipt);
        check_written(.7f, -.8f);
        check(driver.on_deactivate(state) == CallbackReturn::SUCCESS, "final stop");
        check(driver.on_cleanup(state) == CallbackReturn::SUCCESS, "cleanup");
        std::cout << "Production activation ordering, freshness, priming failure and retained-command tests passed\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

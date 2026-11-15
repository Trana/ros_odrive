// Production plugin and installed Jazzy lifecycle. Fake CAN only; no sockets.
#include "../src/odrive_hardware_interface.cpp"
#include "hardware_interface/system.hpp"

#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
using Clock = std::chrono::steady_clock;
using Lifecycle = lifecycle_msgs::msg::State;
const std::vector<int> ids{11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43};
std::mutex bus_mutex;
struct Received { can_frame frame; int64_t receipt; };
std::deque<Received> incoming;
std::vector<can_frame> sent;
std::array<int, 64> drive_state{};
std::array<float, 64> drive_position{};
std::atomic<bool> feedback{false}, silent_idle{false};
std::atomic<int> fail_mode{-1}, fail_prime{-1}, fail_enable{-1};
Clock::time_point last_feedback{};
#ifdef ODRIVE_JAZZY_FAKE_PLUGIN
std::filesystem::path control_dir;
#endif

[[maybe_unused]] void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
int64_t wall_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
[[maybe_unused]] double steady_s() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
template<class Message> void queue(int id, const Message& message) {
    can_frame frame{};
    frame.can_id = (id << 5) | message.cmd_id;
    frame.can_dlc = message.msg_length;
    message.encode_buf(frame.data);
    incoming.push_back({frame, wall_ns()});
}
[[maybe_unused]] std::vector<can_frame> take() {
    std::lock_guard lock(bus_mutex);
    auto result = sent;
    sent.clear();
    return result;
}
[[maybe_unused]] size_t count(const std::vector<can_frame>& frames, uint32_t command) {
    return std::count_if(frames.begin(), frames.end(), [&](const auto& f) { return (f.can_id & 31) == command; });
}
void no_idle(const std::vector<can_frame>& frames) {
    for (const auto& frame : frames)
        check((frame.can_id & 31) != Set_Axis_State_msg_t::cmd_id || frame.data[0] != AXIS_STATE_IDLE,
              "software failure must not send IDLE");
}
[[maybe_unused]] void hold(const std::vector<can_frame>& frames, bool native = false) {
    size_t positions = 0;
    for (const auto& frame : frames) {
        if ((frame.can_id & 31) != Set_Input_Pos_msg_t::cmd_id) continue;
        ++positions;
        Set_Input_Pos_msg_t message{};
        message.decode_buf(frame.data);
        const float expected = native ? -.3 * 9 / (2 * M_PI) : drive_position[frame.can_id >> 5];
        check(std::abs(message.Input_Pos - expected) < 1e-5, "wrong retained position");
        check(message.Vel_FF == 0 && message.Torque_FF == 0, "hold must have zero feedforward");
    }
    check(positions > 0, "expected continuous position support");
    no_idle(frames);
}
} // namespace

bool SocketCanIntf::init(const std::string&, EpollEventLoop*, FrameProcessor processor) {
    frame_processor_ = std::move(processor);
#ifdef ODRIVE_JAZZY_FAKE_PLUGIN
    const char* root = std::getenv("ODRIVE_JAZZY_FAKE_CONTROL_DIR");
    if (!root) return false;
    control_dir = root;
    for (int id : ids) drive_position[id] = .01f * id;
    feedback = true;
#endif
    return true;
}
void SocketCanIntf::deinit() {}
bool SocketCanIntf::read_nonblocking() {
    Received sample;
    {
        std::lock_guard lock(bus_mutex);
#ifdef ODRIVE_JAZZY_FAKE_PLUGIN
        silent_idle = std::filesystem::exists(control_dir / "silent-idle");
#endif
        if (incoming.empty() && feedback && Clock::now() - last_feedback >= std::chrono::milliseconds(10)) {
            last_feedback = Clock::now();
            for (int id : ids) {
                Get_Encoder_Estimates_msg_t encoder{};
                encoder.Pos_Estimate = drive_position[id];
                queue(id, encoder);
                Heartbeat_msg_t heartbeat{};
                heartbeat.Axis_State = drive_state[id];
                if (!(silent_idle && drive_state[id] == AXIS_STATE_IDLE)) queue(id, heartbeat);
                queue(id, Get_Error_msg_t{});
            }
        }
        if (incoming.empty()) return false;
        sample = incoming.front();
        incoming.pop_front();
    }
    last_receive_timestamp_ns_ = sample.receipt;
    frame_processor_(sample.frame);
    return true;
}
bool SocketCanIntf::send_can_frame(const can_frame& frame) {
    std::lock_guard lock(bus_mutex);
    sent.push_back(frame);
    const int node = frame.can_id >> 5, command = frame.can_id & 31;
#ifdef ODRIVE_JAZZY_FAKE_PLUGIN
    fail_enable = std::filesystem::exists(control_dir / "fail-enable") ? 23 : -1;
    silent_idle = std::filesystem::exists(control_dir / "silent-idle");
    std::ofstream(control_dir / "frames.log", std::ios::app)
        << node << ' ' << command << ' ' << static_cast<int>(frame.data[0]) << '\n';
#endif
    if (command == Set_Controller_Mode_msg_t::cmd_id && node == fail_mode) return false;
    if (command == Set_Input_Pos_msg_t::cmd_id && node == fail_prime) return false;
    if (command == Set_Input_Pos_msg_t::cmd_id && drive_state[node] == AXIS_STATE_CLOSED_LOOP_CONTROL) {
        Set_Input_Pos_msg_t position{};
        position.decode_buf(frame.data);
        drive_position[node] = position.Input_Pos;
    }
    if (command == Set_Axis_State_msg_t::cmd_id) {
        if (frame.data[0] == AXIS_STATE_CLOSED_LOOP_CONTROL && node == fail_enable) return false;
        drive_state[node] = frame.data[0];
        if (!(silent_idle && frame.data[0] == AXIS_STATE_IDLE)) {
            Heartbeat_msg_t heartbeat{};
            heartbeat.Axis_State = frame.data[0];
            queue(node, heartbeat);
        }
    }
    return true;
}

#ifndef ODRIVE_JAZZY_FAKE_PLUGIN
int main() {
    try {
        auto implementation = std::make_unique<odrive_ros2_control::ODriveHardwareInterface>();
        auto& driver = *implementation;
        hardware_interface::System system(std::move(implementation));
        hardware_interface::HardwareInfo info{};
        info.name = "jazzy_drives";
        info.type = "system";
        info.hardware_parameters = {{"can", "fake_only"}, {"require_command_receipt", "true"},
            {"can_tx_pacing", "coherent_batch"}, {"can_batch_deadline_hold", "true"},
            {"can_hold_max_snapshot_age_ms", "100"}, {"can_hold_max_completion_span_ms", "50"},
            {"can_hold_max_observation_age_ms", "1000"}, {"can_hold_max_submission_age_ms", "1000"},
            {"can_hold_max_tracking_error_rad", "0.2"}};
        std::vector<std::string> claims;
        for (int id : ids) {
            drive_position[id] = .01f * id;
            hardware_interface::ComponentInfo joint;
            joint.name = "joint_" + std::to_string(id);
            joint.type = "joint";
            joint.parameters["node_id"] = std::to_string(id);
            claims.push_back(joint.name + "/position");
            info.joints.push_back(joint);
        }
        check(system.initialize(info, rclcpp::get_logger("jazzy_test"), std::make_shared<rclcpp::Clock>()).id()
              == Lifecycle::PRIMARY_STATE_UNCONFIGURED, "initialize");
        check(system.configure().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "initial verified IDLE");
        take();
        check(system.activate().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "missing encoders must reject");
        check(take().empty(), "rejected feedback must cause no drive writes");
        feedback = true;
        const rclcpp::Time time(0);
        const rclcpp::Duration period(0, 5000000);
        auto fresh = [&] { std::this_thread::sleep_for(std::chrono::milliseconds(12)); driver.read(time, period); };
        fresh();
        fail_mode = 23;
        check(system.activate().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "failed mode must reject");
        auto frames = take();
        check(count(frames, Set_Input_Pos_msg_t::cmd_id) == 0 && count(frames, Set_Axis_State_msg_t::cmd_id) == 0,
              "failed mode cannot prime or enable");
        fail_mode = -1;
        fail_prime = 23;
        fresh();
        check(system.activate().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "failed prime must reject");
        check(count(take(), Set_Axis_State_msg_t::cmd_id) == 0, "failed prime cannot enable");
        fail_prime = -1;
        fail_enable = 23;
        fresh();
        check(system.activate().id() == Lifecycle::PRIMARY_STATE_UNCONFIGURED,
              "partial enable ERROR must leave retryable UNCONFIGURED, never inactive");
        frames = take();
        check(count(frames, Set_Controller_Mode_msg_t::cmd_id) == ids.size(), "every position mode must be submitted");
        size_t first_enable = 0;
        while (first_enable < frames.size() && (frames[first_enable].can_id & 31) != Set_Axis_State_msg_t::cmd_id)
            ++first_enable;
        check(first_enable == ids.size() * 2, "all modes and primes must precede any enable");
        no_idle(frames);
        // No write(), controller claims, or lifecycle read() in this interval.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        hold(take());
        fail_enable = -1;
        silent_idle = true;
        check(system.configure().id() == Lifecycle::PRIMARY_STATE_UNCONFIGURED,
              "explicit Stop with missing IDLE acknowledgement stays unconfigured");
        frames = take();
        check(count(frames, Set_Axis_State_msg_t::cmd_id) >= ids.size(), "Stop must actually request IDLE");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        check(count(take(), Set_Input_Pos_msg_t::cmd_id) == 0, "failed Stop inhibits both sources");
        silent_idle = false;
        check(system.configure().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "explicit Stop retry verifies IDLE");
        take();
        fresh();
        check(system.activate().id() == Lifecycle::PRIMARY_STATE_ACTIVE, "fresh activation without claims");
        take();
        // Healthy preclaim support continues longer than native producer expiry.
        for (int i = 0; i < 100; ++i) { fresh(); }
        hold(take());
        check(driver.perform_command_mode_switch({claims[0]}, {}) == hardware_interface::return_type::OK,
              "partial position claim");
        auto interfaces = driver.export_command_interfaces();
        auto command = [&](double target, double receipt, bool one_missing = false) {
            for (auto& item : interfaces) {
                if (item.get_interface_name() == "position") (void)item.set_value(target);
                if (item.get_interface_name() == "command_receipt_monotonic_s")
                    (void)item.set_value(one_missing && item.get_prefix_name() == "joint_11" ? NAN : receipt);
            }
            driver.write(time, period);
            std::this_thread::sleep_for(std::chrono::milliseconds(35));
            driver.read(time, period);
        };
        command(.3, steady_s()); hold(take());
        driver.perform_command_mode_switch(claims, {});
        command(.3, NAN); hold(take());
        command(.3, steady_s() + 10); hold(take());
        command(.3, steady_s() - 10); hold(take());
        command(.3, steady_s(), true); hold(take());
        command(.3, steady_s());
        take(); // Contains the final measured round and first complete native round.
        command(.3, steady_s()); hold(take(), true);
        check(driver.perform_command_mode_switch({"joint_11/velocity"}, {claims[0]})
              == hardware_interface::return_type::ERROR, "velocity-only takeover must reject atomically");
        command(.3, steady_s()); hold(take(), true);
        driver.perform_command_mode_switch({}, claims);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        driver.read(time, period);
        hold(take(), true); // Never jump back to the initial activation pose.
        for (const auto& item : driver.export_state_interfaces())
            if (item.get_interface_name() == "support_state")
                check(item.get_optional().value_or(NAN)
                          == static_cast<double>(odrive_ros2_control::SupportFallback::State::holding),
                      "retired native producer must expose recovery hold to the supervisor");
        driver.perform_command_mode_switch(claims, {});
        command(.5, steady_s()); hold(take(), true); // A retired producer cannot resume without Rest.
        check(system.deactivate().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "normal explicit Stop");
        take();
        check(system.cleanup().id() == Lifecycle::PRIMARY_STATE_UNCONFIGURED, "cleanup");
        std::cout << "Jazzy no-claim support, partial enable ERROR, real lifecycle Stop/retry and native transfer passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
#endif

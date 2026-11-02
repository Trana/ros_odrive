// Exercise the production plugin with a fake transport, never SocketCAN.
#include "../src/odrive_hardware_interface.cpp"
#include "hardware_interface/system.hpp"
#include <deque>
#include <iostream>
#include <stdexcept>

namespace {
std::deque<can_frame> incoming;
std::atomic<int> position_sends{0};
int idle_sends = 0;
bool silent_drive = false;
bool fail_send = false;
std::atomic<bool> fail_position{false};
std::atomic<int> stall_position_at{-1};

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename System>
const rclcpp_lifecycle::State& initialize(System& system, const hardware_interface::HardwareInfo& info) {
    // Humble and Jazzy expose different initialization signatures.
    if constexpr (requires { system.initialize(info); }) {
        return system.initialize(info);
    } else {
        return system.initialize(info, rclcpp::get_logger("shutdown_test"),
            std::make_shared<rclcpp::Clock>());
    }
}
}

bool SocketCanIntf::init(const std::string&, EpollEventLoop*, FrameProcessor processor) {
    frame_processor_ = std::move(processor);
    return true;
}
void SocketCanIntf::deinit() {}
bool SocketCanIntf::read_nonblocking() {
    if (incoming.empty()) return false;
    auto frame = incoming.front();
    incoming.pop_front();
    last_receive_timestamp_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame_processor_(frame);
    return true;
}
bool SocketCanIntf::send_can_frame(const can_frame& frame) {
    const auto command = frame.can_id & 0x1f;
    const auto id = frame.can_id >> 5;
    if (command == Set_Input_Pos_msg_t::cmd_id) {
        const auto count = ++position_sends;
        int expected = count;
        if (stall_position_at.compare_exchange_strong(expected, -1))
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        if (fail_position) return false;
    }
    if (command == Set_Axis_State_msg_t::cmd_id && frame.data[0] == AXIS_STATE_IDLE) {
        ++idle_sends;
        if (fail_send && id == 43) return false;
        if (!(silent_drive && id == 43)) {
            can_frame reply{};
            reply.can_id = (id << 5) | Heartbeat_msg_t::cmd_id;
            reply.can_dlc = 8;
            reply.data[4] = AXIS_STATE_IDLE;
            incoming.push_back(reply);
        }
    }
    return true;
}

int main() {
    try {
        // Test the production decoder without a CAN socket or physical motion.
        odrive_ros2_control::Axis axis(nullptr, 11, rclcpp::get_logger("encoder_test"),
                                      false, rclcpp::Logger::Level::Info);
        for (const float motor_velocity : {-2.0f, 0.0f, 2.0f}) {
            can_frame frame{};
            frame.can_id = (11 << 5) | Get_Encoder_Estimates_msg_t::cmd_id;
            frame.can_dlc = Get_Encoder_Estimates_msg_t::msg_length;
            Get_Encoder_Estimates_msg_t sample;
            sample.Pos_Estimate = 0.25f;
            sample.Vel_Estimate = motor_velocity;
            sample.encode_buf(frame.data);
            axis.on_can_msg(rclcpp::Time(0), frame);
            const double first_position = axis.pos_estimate_;
            sample.Pos_Estimate += motor_velocity * 0.02f;
            sample.encode_buf(frame.data);
            axis.on_can_msg(rclcpp::Time(0), frame);
            const double derivative = (axis.pos_estimate_ - first_position) / 0.02;
            check(std::abs(axis.vel_estimate_ - derivative) < 1e-5,
                  "joint velocity must match the derivative of converted joint position");
        }
        auto implementation = std::make_unique<odrive_ros2_control::ODriveHardwareInterface>();
        auto& driver = *implementation;
        hardware_interface::System system(std::move(implementation));
        using Lifecycle = lifecycle_msgs::msg::State;
        hardware_interface::HardwareInfo info;
        info.name = "test_drives";
        info.type = "system";
        info.hardware_parameters["can"] = "fake_transport_only";
        if (std::getenv("ODRIVE_TEST_PHASE_LOCKED"))
            info.hardware_parameters["can_tx_pacing"] = "phase_locked";
        if (std::getenv("ODRIVE_TEST_BATCH"))
            info.hardware_parameters["can_tx_pacing"] = "coherent_batch";
        if (std::getenv("ODRIVE_TEST_BATCH_GAP"))
            info.hardware_parameters["can_tx_batch_gap_us"] = "500";
        if (std::getenv("ODRIVE_TEST_HOLD")) {
            info.hardware_parameters["can_batch_deadline_hold"] = "true";
            info.hardware_parameters["can_hold_max_snapshot_age_ms"] = "100";
            info.hardware_parameters["can_hold_max_completion_span_ms"] = "50";
            info.hardware_parameters["can_hold_max_observation_age_ms"] = "1000";
            info.hardware_parameters["can_hold_max_submission_age_ms"] = "1000";
            info.hardware_parameters["can_hold_max_tracking_error_rad"] = "0.2";
        }
        std::vector<int> ids{11, 43};
        if (std::getenv("ODRIVE_TEST_TWELVE_AXES"))
            ids = {11, 43, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
        std::vector<std::string> interfaces;
        for (const auto id : ids) {
            hardware_interface::ComponentInfo joint;
            joint.name = "joint_" + std::to_string(id);
            joint.type = "joint";
            joint.parameters["node_id"] = std::to_string(id);
            info.joints.push_back(joint);
            interfaces.push_back(joint.name + "/position");
        }
        check(initialize(system, info).id() == Lifecycle::PRIMARY_STATE_UNCONFIGURED, "initialize");
        check(system.configure().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "configure");
        check(idle_sends == static_cast<int>(ids.size()), "configure must establish confirmed idle for every drive");
        driver.perform_command_mode_switch(interfaces, {});
        driver.write(rclcpp::Time(0), rclcpp::Duration(0, 5000000));
        check(position_sends == 0, "inactive startup must inhibit position writes");
        for (const auto id : ids) {
            can_frame encoder{};
            encoder.can_id = (id << 5) | Get_Encoder_Estimates_msg_t::cmd_id;
            encoder.can_dlc = Get_Encoder_Estimates_msg_t::msg_length;
            Get_Encoder_Estimates_msg_t sample{};
            sample.Pos_Estimate = .25f;
            sample.encode_buf(encoder.data);
            incoming.push_back(encoder);
        }
        driver.read(rclcpp::Time(0), rclcpp::Duration(0, 5000000));
        check(system.activate().id() == Lifecycle::PRIMARY_STATE_ACTIVE, "activate");
        driver.perform_command_mode_switch(interfaces, {});
        const rclcpp::Time time(0);
        const rclcpp::Duration period(0, 5000000);
        driver.write(time, period);
        for (int i = 0; i < 100 && position_sends < static_cast<int>(ids.size()); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(position_sends >= static_cast<int>(ids.size()), "normal active driver must write targets asynchronously");
        if (std::getenv("ODRIVE_TEST_HOLD")) {
            const auto refresh_feedback = [&] {
            for (const auto id : ids) {
                can_frame heartbeat{};
                heartbeat.can_id = (id << 5) | Heartbeat_msg_t::cmd_id;
                heartbeat.can_dlc = Heartbeat_msg_t::msg_length;
                Heartbeat_msg_t heartbeat_sample{};
                heartbeat_sample.Axis_State = AXIS_STATE_CLOSED_LOOP_CONTROL;
                heartbeat_sample.encode_buf(heartbeat.data);
                incoming.push_back(heartbeat);

                can_frame error{};
                error.can_id = (id << 5) | Get_Error_msg_t::cmd_id;
                error.can_dlc = Get_Error_msg_t::msg_length;
                Get_Error_msg_t error_sample{};
                error_sample.encode_buf(error.data);
                incoming.push_back(error);

                can_frame encoder{};
                encoder.can_id = (id << 5) | Get_Encoder_Estimates_msg_t::cmd_id;
                encoder.can_dlc = Get_Encoder_Estimates_msg_t::msg_length;
                Get_Encoder_Estimates_msg_t encoder_sample{};
                encoder_sample.Pos_Estimate = .25f;
                encoder_sample.encode_buf(encoder.data);
                incoming.push_back(encoder);
            }
            driver.read(time, period);
            };
            refresh_feedback();
            driver.write(time, period);
            const auto idle_before_stall = idle_sends;
            const auto sends_before_stall = position_sends.load();
            stall_position_at = sends_before_stall + 4;
            driver.write(time, period);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            check(idle_sends == idle_before_stall,
                  "recoverable batch deadline must not request IDLE");
            check(position_sends > sends_before_stall + 4,
                  "recoverable batch deadline must continue position-only hold sends");
            check(driver.write(time, period) == hardware_interface::return_type::OK,
                  "new producer writes must be fenced without failing the active hold");
            for (int refresh = 0; refresh < 12; ++refresh) {
                refresh_feedback();
                const auto before = position_sends.load();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                check(idle_sends == idle_before_stall && position_sends > before,
                      "healthy hold must continue beyond the old one-second timeout");
                check(driver.write(time, period) == hardware_interface::return_type::OK,
                      "continued hold must fence new motion writes");
            }
        }
        if (ids.size() == 12) {
            const auto begin = std::chrono::steady_clock::now();
            for (int i = 0; i < 100; ++i) {
                driver.read(time, period);
                driver.write(time, period);
            }
            const auto elapsed = std::chrono::steady_clock::now() - begin;
            std::cout << "100 twelve-axis fake read/write cycles: "
                      << std::chrono::duration<double, std::milli>(elapsed).count() << " ms\n";
            check(elapsed < std::chrono::milliseconds(300), "write must not retain per-axis sleeps");
            for (int i = 0; i < 1000; ++i) incoming.push_back(can_frame{});
            driver.read(time, period);
            check(incoming.size() >= 744, "read must bound backlog processing to at most 256 frames");
            incoming.clear();
        }

        silent_drive = true;
        const auto start = std::chrono::steady_clock::now();
        check(system.deactivate().id() == Lifecycle::PRIMARY_STATE_ACTIVE,
            "partial acknowledgement must leave lifecycle active for retry");
        check(std::chrono::steady_clock::now() - start < std::chrono::seconds(2),
            "shutdown must fit service timeout");
        check(idle_sends > 2, "driver must retry unanswered requests");
        const auto stopped_sends = position_sends.load();
        driver.write(time, period);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        check(position_sends == stopped_sends, "failed shutdown must inhibit motion writes");

        // A failed send must not become lifecycle success either.
        silent_drive = false;
        fail_send = true;
        check(system.deactivate().id() == Lifecycle::PRIMARY_STATE_ACTIVE,
            "send failure must leave lifecycle active for retry");
        driver.write(time, period);
        check(position_sends == stopped_sends, "send failure must keep writes inhibited");

        fail_send = false;
        check(system.deactivate().id() == Lifecycle::PRIMARY_STATE_INACTIVE,
            "later Rest retry must recover once every drive confirms idle");
        driver.write(time, period);
        check(position_sends == stopped_sends, "successful shutdown must keep writes inhibited");
        for (const auto id : ids) {
            can_frame encoder{};
            encoder.can_id = (id << 5) | Get_Encoder_Estimates_msg_t::cmd_id;
            encoder.can_dlc = Get_Encoder_Estimates_msg_t::msg_length;
            Get_Encoder_Estimates_msg_t sample{};
            sample.Pos_Estimate = .25f;
            sample.encode_buf(encoder.data);
            incoming.push_back(encoder);
        }
        driver.read(rclcpp::Time(0), rclcpp::Duration(0, 5000000));
        check(system.activate().id() == Lifecycle::PRIMARY_STATE_ACTIVE, "explicit reactivation");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        check(position_sends == stopped_sends + static_cast<int>(ids.size()), "reactivation must only prime measured targets");
        driver.write(time, period);
        for (int i = 0; i < 100 && position_sends == stopped_sends; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(position_sends > stopped_sends, "explicit reactivation restores motion writes");
        fail_position = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        check(driver.write(time, period) == hardware_interface::return_type::ERROR,
              "transport failure must latch and surface to ros2_control");
        const auto failed_sends = position_sends.load();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        check(position_sends == failed_sends, "fault must stop further target sends");
        fail_position = false;
        check(system.deactivate().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "final deactivate");
        for (const auto id : ids) {
            can_frame encoder{};
            encoder.can_id = (id << 5) | Get_Encoder_Estimates_msg_t::cmd_id;
            encoder.can_dlc = Get_Encoder_Estimates_msg_t::msg_length;
            Get_Encoder_Estimates_msg_t sample{};
            sample.Pos_Estimate = .25f;
            sample.encode_buf(encoder.data);
            incoming.push_back(encoder);
        }
        driver.read(rclcpp::Time(0), rclcpp::Duration(0, 5000000));
        check(system.activate().id() == Lifecycle::PRIMARY_STATE_ACTIVE, "stale test activation");
        driver.write(time, period);
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        check(driver.read(time, period) == hardware_interface::return_type::ERROR,
              "stale producer must latch and surface to ros2_control");
        const auto stale_sends = position_sends.load();
        driver.write(time, period);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        check(position_sends == stale_sends, "fresh writes must not silently recover a stale fault");
        check(system.deactivate().id() == Lifecycle::PRIMARY_STATE_INACTIVE, "stale recovery Rest");
        check(system.cleanup().id() == Lifecycle::PRIMARY_STATE_UNCONFIGURED, "cleanup");
        std::cout << "Production driver shutdown fault tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

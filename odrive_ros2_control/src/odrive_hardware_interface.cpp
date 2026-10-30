#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"
#include "verified_idle_shutdown.hpp"
#include "hardware_timing.hpp"
#include "encoder_timing.hpp"
#include "paced_commands.hpp"
#include <cstdlib>
#include <algorithm>
#include <mutex>
#include <thread>
#include <cmath>

namespace odrive_ros2_control {

class NegateWrapper {
public:
    NegateWrapper(double& value) : value_(value) {}

    double get() const { return -value_; }

private:
    double& value_;
};

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    ODriveHardwareInterface()
        : logger_(rclcpp::get_logger("ODriveHardwareInterface")),
          enable_logging_(false), // Set to false to disable logging
          log_level_(rclcpp::Logger::Level::Info) {} // Set default log level

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);

    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
    rclcpp::Logger logger_;
    bool enable_logging_;
    rclcpp::Logger::Level log_level_;
    std::mutex can_mutex_;
    bool shutdown_requested_ = true;
    VerifiedIdleShutdown* idle_shutdown_ = nullptr;
    bool timing_enabled_ = false;
    bool export_encoder_receipt_timing_ = false;
    bool require_command_receipt_ = false;
    double activation_monotonic_s_ = 0.0;
    HardwareTiming timing_;
    // Last member: join before timing counters or logger are destroyed.
    std::jthread timing_reporter_;
    PacedCommands commands_;
    // Joined before any state used by the sender is destroyed.
    std::jthread command_sender_;
    std::optional<PacedCommands::Clock::time_point> previous_send_attempt_;

    void log_message(rclcpp::Logger::Level level, const std::string& message) const;
    PacedCommands::Clock::time_point sender_step();
    void stop_sender_on_fault(); // Caller holds can_mutex_.
};

void ODriveHardwareInterface::log_message(rclcpp::Logger::Level level, const std::string& message) const {
    if (!enable_logging_ || level < log_level_) {
        return;
    }

    switch (level) {
        case rclcpp::Logger::Level::Debug: RCLCPP_DEBUG(logger_, message.c_str()); break;
        case rclcpp::Logger::Level::Info: RCLCPP_INFO(logger_, message.c_str()); break;
        case rclcpp::Logger::Level::Warn: RCLCPP_WARN(logger_, message.c_str()); break;
        case rclcpp::Logger::Level::Error: RCLCPP_ERROR(logger_, message.c_str()); break;
        case rclcpp::Logger::Level::Fatal: RCLCPP_FATAL(logger_, message.c_str()); break;
        default: break;
    }
}

struct Axis {
    Axis(
        SocketCanIntf* can_intf,
        uint32_t node_id,
        const rclcpp::Logger& logger,
        bool enable_logging,
        rclcpp::Logger::Level log_level
    )
        : can_intf_(can_intf),
          node_id_(node_id),
          logger_(logger),
          enable_logging_(enable_logging),
          log_level_(log_level) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);
    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;
    rclcpp::Logger logger_;
    bool enable_logging_;
    rclcpp::Logger::Level log_level_;

    double pos_setpoint_ = 0.0f;
    double activation_hold_ = NAN;
    double command_receipt_monotonic_s_ = NAN;
    double vel_setpoint_ = 0.0f;
    double torque_setpoint_ = 0.0f;
    double pos_estimate_ = NAN;
    double vel_estimate_ = NAN;
    double torque_target_ = NAN;
    double torque_estimate_ = NAN;
    std::shared_ptr<EncoderTiming> encoder_timing_ = std::make_shared<EncoderTiming>();
    double encoder_receipt_monotonic_s_ = NAN;

    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    template <typename T>
    bool send(const T& msg) {
        struct can_frame frame{};
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        if (!can_intf_->send_can_frame(frame)) {
            if (enable_logging_ && log_level_ <= rclcpp::Logger::Level::Error) {
                RCLCPP_ERROR(
                    logger_,
                    "Failed to send CAN frame: Node ID: %d, Command ID: %d, Frame ID: 0x%X, DLC: %d, Error: %s",
                    node_id_,
                    msg.cmd_id,
                    frame.can_id,
                    frame.can_dlc,
                    strerror(errno)
                );
            }
            return false;
        } else {
            if (enable_logging_ && log_level_ <= rclcpp::Logger::Level::Info) {
                RCLCPP_INFO(
                    logger_,
                    "Successfully sent CAN frame: Node ID: %d, Command ID: %d, Frame ID: 0x%X, DLC: %d",
                    node_id_,
                    msg.cmd_id,
                    frame.can_id,
                    frame.can_dlc
                );
            }
        }
        return true;
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];
    const char* tracing = std::getenv("MARVIN_TRACING_MODE");
    const std::string tracing_mode = tracing ? tracing : "metrics";
    if (tracing_mode != "off" && tracing_mode != "metrics" && tracing_mode != "capture") {
        RCLCPP_ERROR(logger_, "Invalid MARVIN_TRACING_MODE");
        return CallbackReturn::ERROR;
    }
    timing_enabled_ = tracing_mode != "off";
    const auto receipt_option = info_.hardware_parameters.find("export_encoder_receipt_timing");
    if (receipt_option != info_.hardware_parameters.end()) {
        if (receipt_option->second != "true" && receipt_option->second != "false") {
            RCLCPP_ERROR(logger_, "export_encoder_receipt_timing must be true or false");
            return CallbackReturn::ERROR;
        }
        export_encoder_receipt_timing_ = receipt_option->second == "true";
    }
    if (export_encoder_receipt_timing_ && !timing_enabled_) {
        RCLCPP_ERROR(logger_, "Encoder receipt export requires tracing metrics or capture");
        return CallbackReturn::ERROR;
    }
    const auto command_receipt = info_.hardware_parameters.find("require_command_receipt");
    if (command_receipt != info_.hardware_parameters.end()) {
        if (command_receipt->second != "true" && command_receipt->second != "false")
            return CallbackReturn::ERROR;
        require_command_receipt_ = command_receipt->second == "true";
    }
    commands_.set_batch_timing(timing_enabled_ ? &timing_.batch : nullptr);

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")), logger_, enable_logging_, log_level_);
    }
    try {
        auto parameter = [&](const char* name, double fallback) {
            const auto found = info_.hardware_parameters.find(name);
            if (found == info_.hardware_parameters.end()) return fallback;
            size_t consumed = 0;
            const auto value = std::stod(found->second, &consumed);
            if (consumed != found->second.size()) throw std::invalid_argument(name);
            return value;
        };
        const auto mode_it = info_.hardware_parameters.find("can_tx_pacing");
        const auto mode = mode_it == info_.hardware_parameters.end() ? "relative" : mode_it->second;
        if (mode != "relative" && mode != "phase_locked" && mode != "bounded_phase" && mode != "coherent_batch")
            throw std::invalid_argument("Unknown can_tx_pacing mode");
        const auto batch_gap_us = parameter("can_tx_batch_gap_us", 0);
        commands_.configure(axes_.size(), parameter("can_tx_rate_hz", 50),
                            parameter("can_command_stale_ms", 250), mode == "phase_locked",
                            mode == "bounded_phase", mode == "coherent_batch", batch_gap_us);
        RCLCPP_INFO(logger_, "CAN sender pacing: %s, batch gap: %.1f us", mode.c_str(), batch_gap_us);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(logger_, "Invalid CAN sender configuration: %s", error.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State& previous_state) {
    can_intf_.collect_receive_timestamps = true;
    for (auto& axis : axes_) {
        axis.encoder_timing_->reset_receipt();
        axis.encoder_receipt_monotonic_s_ = NAN;
    }
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        log_message(rclcpp::Logger::Level::Error, "Failed to initialize SocketCAN on " + can_intf_name_);
        return CallbackReturn::ERROR;
    }
    log_message(rclcpp::Logger::Level::Info, "Initialized SocketCAN on " + can_intf_name_);
    // Establish a verified idle baseline before reporting INACTIVE. Starting
    // processes must not adopt a motor left enabled by an earlier process.
    const auto result = on_deactivate(previous_state);
    if (result == CallbackReturn::SUCCESS && !command_sender_.joinable()) {
        command_sender_ = std::jthread([this](std::stop_token stop) {
            while (!stop.stop_requested()) {
                const auto deadline = sender_step();
                // Outside transport mutex: reads and updates continue while pacing.
                std::this_thread::sleep_until(deadline);
                if (timing_enabled_ && !stop.stop_requested()) {
                    const auto late = PacedCommands::Clock::now() - deadline;
                    timing_.sender_wake_lateness.add(static_cast<uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::max(late, PacedCommands::Clock::duration::zero())).count()));
                }
            }
        });
    }
    if (result == CallbackReturn::SUCCESS && timing_enabled_ && !timing_reporter_.joinable()) {
        timing_reporter_ = std::jthread([this](std::stop_token stop) {
            auto previous = std::chrono::steady_clock::now();
            while (!stop.stop_requested()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                const auto now = std::chrono::steady_clock::now();
                const auto seconds = std::chrono::duration<double>(now - previous).count();
                if (seconds >= 1.0 && !stop.stop_requested()) {
                    // Non-control thread only; diagnostic logging never holds can_mutex_.
                    RCLCPP_INFO(logger_, "HARDWARE_TIMING %s", timing_.snapshot(seconds).c_str());
                    std::ostringstream encoder_report;
                    encoder_report << "{\"schema_version\":\"encoder-timing.v1\",\"window_s\":"
                                   << seconds << ",\"motors\":[";
                    bool first = true;
                    for (auto& axis : axes_) {
                        if (!first) encoder_report << ',';
                        first = false;
                        axis.encoder_timing_->json(encoder_report, axis.node_id_);
                    }
                    encoder_report << "]}";
                    RCLCPP_INFO(logger_, "ENCODER_TIMING %s", encoder_report.str().c_str());
                    previous = now;
                }
            }
        });
    }
    return result;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    {
        std::lock_guard<std::mutex> lock(can_mutex_);
        shutdown_requested_ = true;
        commands_.inhibit();
    }
    if (command_sender_.joinable()) {
        command_sender_.request_stop();
        command_sender_.join();
    }
    if (timing_reporter_.joinable()) {
        timing_reporter_.request_stop();
        timing_reporter_.join();
    }
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    commands_.inhibit();
    const auto now = PacedCommands::Clock::now();
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const double now_s = static_cast<double>(steady_ns) / 1e9;
    for (auto& axis : axes_) {
        axis.encoder_timing_->sample(wall_ns, steady_ns);
        const auto receipt = axis.encoder_timing_->receipt_monotonic_seconds();
        if ((require_command_receipt_ && !axis.pos_input_enabled_) ||
            !std::isfinite(axis.pos_estimate_) || !std::isfinite(receipt) ||
            receipt > now_s || now_s - receipt > 0.1) {
            RCLCPP_ERROR(logger_, "Activation rejected: missing or stale encoder on node %u", axis.node_id_);
            return CallbackReturn::FAILURE;
        }
    }
    for (auto& axis : axes_) {
        axis.activation_hold_ = axis.pos_setpoint_ = axis.pos_estimate_;
        axis.vel_setpoint_ = axis.torque_setpoint_ = 0.0;
        Set_Input_Pos_msg_t hold{};
        hold.Input_Pos = -(axis.activation_hold_ * 9.0 / (2.0 * M_PI));
        if (!axis.send(hold)) {
            RCLCPP_ERROR(logger_, "Activation rejected: hold prime failed on node %u", axis.node_id_);
            return CallbackReturn::FAILURE;
        }
    }
    activation_monotonic_s_ = std::chrono::duration<double>(
        PacedCommands::Clock::now().time_since_epoch()).count();
    for (auto& axis : axes_) {
        Set_Axis_State_msg_t msg{};
        msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;
        if (!axis.send(msg)) {
            RCLCPP_ERROR(logger_, "Activation unconfirmed: closed-loop send failed on node %u", axis.node_id_);
            return CallbackReturn::FAILURE;
        }
    }
    shutdown_requested_ = false;
    commands_.activate(PacedCommands::Clock::now());

    // Activation is explicit; command-mode switching only selects input modes.
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    // Keep writes inhibited even on FAILURE: remaining in lifecycle ACTIVE
    // must not resume motion while a later Rest attempt retries shutdown.
    shutdown_requested_ = true;
    commands_.inhibit();
    using Clock = std::chrono::steady_clock;
    // Discard pre-request heartbeats. Bound draining so a busy bus cannot make
    // the lifecycle service exceed the controller's two-second timeout.
    const auto drain_deadline = Clock::now() + std::chrono::milliseconds(20);
    bool drained = false;
    while (Clock::now() < drain_deadline) {
        if (!can_intf_.read_nonblocking()) {
            drained = true;
            break;
        }
    }
    std::vector<uint32_t> node_ids;
    for (const auto& axis : axes_) node_ids.push_back(axis.node_id_);
    VerifiedIdleShutdown shutdown(node_ids);
    // If the backlog cannot be drained, still send idle requests, but do not
    // accept potentially old heartbeats as acknowledgement for this attempt.
    idle_shutdown_ = drained ? &shutdown : nullptr;
    const bool verified = shutdown.run(
        [this](uint32_t node_id) {
            Set_Axis_State_msg_t msg;
            msg.Axis_Requested_State = AXIS_STATE_IDLE;
            for (auto& axis : axes_) {
                if (axis.node_id_ == node_id) return axis.send(msg);
            }
            return false;
        },
        [this]() {
            for (int i = 0; i < 256 && can_intf_.read_nonblocking(); ++i) {}
        },
        []() { return Clock::now(); },
        [](auto delay) { std::this_thread::sleep_for(delay); });
    idle_shutdown_ = nullptr;
    if (!verified) {
        for (const auto& drive : shutdown.drives()) {
            if (!drive.idle_confirmed) {
                RCLCPP_ERROR(logger_,
                    "Shutdown UNCONFIRMED: ODrive %u, idle request sent=%s, send failures=%zu. "
                    "Motor may remain enabled; motion writes inhibited. Retry Rest.",
                    drive.node_id, drive.request_sent ? "yes" : "no", drive.send_failures);
            }
        }
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(logger_, "Shutdown verified: all %zu ODrives reported IDLE.", axes_.size());
    commands_.clear_fault_after_idle();
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (export_encoder_receipt_timing_) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "encoder_receipt_monotonic_s",
                &axes_[i].encoder_receipt_monotonic_s_));
        }
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (require_command_receipt_) {
            command_interfaces.emplace_back(info_.joints[i].name,
                "command_receipt_monotonic_s", &axes_[i].command_receipt_monotonic_s_);
        }
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    commands_.discard(); // No old-mode setpoint may follow a mode change.
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}
        };

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            Set_Controller_Mode_msg_t msg;
            if (axis.pos_input_enabled_) {
                log_message(rclcpp::Logger::Level::Info, "Setting " + info_.joints[i].name + " to position control");
                msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
                msg.Input_Mode = INPUT_MODE_POS_FILTER;
            } else if (axis.vel_input_enabled_) {
                log_message(rclcpp::Logger::Level::Info, "Setting " + info_.joints[i].name + " to velocity control");
                msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
                msg.Input_Mode = INPUT_MODE_POS_FILTER;
            } else {
                log_message(rclcpp::Logger::Level::Info, "Setting " + info_.joints[i].name + " to torque control");
                msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
                msg.Input_Mode = INPUT_MODE_POS_FILTER;
            }

            bool any_enabled = axis.pos_input_enabled_ || axis.vel_input_enabled_ || axis.torque_input_enabled_;

            if (any_enabled) {
                axis.send(msg); // Set control mode
            }
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    TimedScope total(timing_enabled_ ? &timing_.read : nullptr);
    std::unique_lock<std::mutex> lock(can_mutex_, std::defer_lock);
    {
        TimedScope wait(timing_enabled_ ? &timing_.read_lock : nullptr);
        lock.lock();
    }
    timestamp_ = timestamp;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
    size_t frames = 0;
    while (frames < 256 && can_intf_.read_nonblocking()) {
        ++frames;
        if (timing_enabled_) timing_.rx_frames.fetch_add(1, std::memory_order_relaxed);
        if (std::chrono::steady_clock::now() >= deadline) break;
    }
    if (timing_enabled_ && (frames == 256 || std::chrono::steady_clock::now() >= deadline))
        timing_.rx_budget_hits.fetch_add(1, std::memory_order_relaxed);

    if (timing_enabled_) {
        const auto steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        for (auto& axis : axes_) {
            axis.encoder_timing_->sample(wall, steady);
            if (export_encoder_receipt_timing_)
                axis.encoder_receipt_monotonic_s_ = axis.encoder_timing_->receipt_monotonic_seconds();
        }
    }
    return commands_.faulted() ? return_type::ERROR : return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    TimedScope total(timing_enabled_ ? &timing_.write : nullptr);
    std::unique_lock<std::mutex> lock(can_mutex_, std::defer_lock);
    {
        TimedScope wait(timing_enabled_ ? &timing_.write_lock : nullptr);
        lock.lock();
    }
    if (commands_.faulted()) return return_type::ERROR;
    if (shutdown_requested_) return return_type::OK;
    for (auto& axis : axes_) {
        if (require_command_receipt_ && !(std::isfinite(axis.command_receipt_monotonic_s_) &&
            axis.command_receipt_monotonic_s_ > activation_monotonic_s_)) {
            axis.pos_setpoint_ = axis.activation_hold_;
            axis.vel_setpoint_ = axis.torque_setpoint_ = 0.0;
        }
        if ((axis.pos_input_enabled_ && !std::isfinite(axis.pos_setpoint_)) ||
            (axis.vel_input_enabled_ && !std::isfinite(axis.vel_setpoint_)) ||
            (axis.torque_input_enabled_ && !std::isfinite(axis.torque_setpoint_))) {
            commands_.fault(PacedCommands::FaultReason::non_finite_target);
            stop_sender_on_fault();
            return return_type::ERROR;
        }
    }
    size_t slot = 0;
    const auto stage = [this, &slot](Axis& axis, const auto& msg) {
        can_frame frame{};
        frame.can_id = axis.node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);
        commands_.set(slot, frame);
    };
    for (auto& axis : axes_) {
        // Log the state of control inputs
        if (enable_logging_ && log_level_ <= rclcpp::Logger::Level::Debug) log_message(
            rclcpp::Logger::Level::Debug,
            "Axis " + std::to_string(axis.node_id_)
                + " - pos_input_enabled_: " + std::to_string(axis.pos_input_enabled_)
                + ", vel_input_enabled_: " + std::to_string(axis.vel_input_enabled_) + ", torque_input_enabled_: "
                + std::to_string(axis.torque_input_enabled_) + ", pos_setpoint_: " + std::to_string(axis.pos_setpoint_)
        );

        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            // ODrive references rotation from the encoder, so here it'll be CCW as positive, which is generally the common convention (CCW is positive in rotation vectors)
            constexpr double RAD_TO_TURNS = 1.0 / (2.0 * M_PI);
            constexpr double GR = 9.0; // Gear ratio

            msg.Input_Pos = -(axis.pos_setpoint_ * RAD_TO_TURNS * GR);
            msg.Vel_FF    = axis.vel_input_enabled_ ? (axis.vel_setpoint_ * RAD_TO_TURNS * GR) : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;

            if (enable_logging_ && log_level_ <= rclcpp::Logger::Level::Info) log_message(
                rclcpp::Logger::Level::Info,
                "Sending Set_Input_Pos to Node ID: " + std::to_string(axis.node_id_)
                    + ", Input_Pos: " + std::to_string(msg.Input_Pos) + ", Vel_FF: " + std::to_string(msg.Vel_FF)
                    + ", Torque_FF: " + std::to_string(msg.Torque_FF)
            );

            stage(axis, msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = axis.vel_setpoint_ / (2 * M_PI);
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;

            if (enable_logging_ && log_level_ <= rclcpp::Logger::Level::Info) log_message(
                rclcpp::Logger::Level::Info,
                "Sending Set_Input_Vel to Node ID: " + std::to_string(axis.node_id_) + ", Input_Vel: "
                    + std::to_string(msg.Input_Vel) + ", Input_Torque_FF: " + std::to_string(msg.Input_Torque_FF)
            );

            stage(axis, msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_;

            if (enable_logging_ && log_level_ <= rclcpp::Logger::Level::Info) log_message(
                rclcpp::Logger::Level::Info,
                "Sending Set_Input_Torque to Node ID: " + std::to_string(axis.node_id_)
                    + ", Input_Torque: " + std::to_string(msg.Input_Torque)
            );

            stage(axis, msg);
        } else {
            commands_.set(slot, std::nullopt);
            if (enable_logging_ && log_level_ <= rclcpp::Logger::Level::Warn) log_message(
                rclcpp::Logger::Level::Warn,
                "No control input enabled for Axis " + std::to_string(axis.node_id_) + " - No setpoint sent"
            );
        }
        ++slot;
    }
    commands_.commit(PacedCommands::Clock::now());

    return return_type::OK;
}

PacedCommands::Clock::time_point ODriveHardwareInterface::sender_step() {
    // A busy lifecycle/read callback never stalls the sender into a queued burst.
    std::unique_lock<std::mutex> lock(can_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        if (timing_enabled_) timing_.sender_busy_skips.fetch_add(1, std::memory_order_relaxed);
        return PacedCommands::Clock::now() + commands_.slot();
    }
    if (shutdown_requested_) {
        previous_send_attempt_.reset();
        return PacedCommands::Clock::now() + commands_.slot();
    }
    const auto opportunity = PacedCommands::Clock::now();
    const auto frame = commands_.due(opportunity);
    if (frame) {
        if (timing_enabled_) {
            timing_.target_age_at_send.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                commands_.target_age(opportunity)).count());
            if (previous_send_attempt_) {
                timing_.send_attempt_interval.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    opportunity - *previous_send_attempt_).count());
            }
            previous_send_attempt_ = opportunity;
        }
        TimedScope duration(timing_enabled_ ? &timing_.sender : nullptr);
        bool sent;
        {
            SendExecutionScope execution(timing_enabled_ ? &timing_.send_execution : nullptr);
            sent = can_intf_.send_can_frame(*frame);
        }
        if (timing_enabled_) {
            timing_.tx_attempts.fetch_add(1, std::memory_order_relaxed);
            if (!sent) timing_.tx_failures.fetch_add(1, std::memory_order_relaxed);
        }
        if (!sent) commands_.fault(PacedCommands::FaultReason::send_failure);
    }
    if (!frame) previous_send_attempt_.reset();
    const auto completed = PacedCommands::Clock::now();
    if (!commands_.faulted())
        commands_.finish_submission(completed, frame.has_value());
    if (commands_.faulted()) stop_sender_on_fault();
    return commands_.wake_after(commands_.faulted() ? PacedCommands::Clock::now() : completed);
}

void ODriveHardwareInterface::stop_sender_on_fault() {
    if (!shutdown_requested_) {
        if (timing_enabled_) timing_.sender_faults.fetch_add(1, std::memory_order_relaxed);
        shutdown_requested_ = true;
        // Best effort only. Rest still must verify fresh IDLE acknowledgements.
        for (auto& axis : axes_) {
            Set_Axis_State_msg_t idle;
            idle.Axis_Requested_State = AXIS_STATE_IDLE;
            axis.send(idle);
        }
        RCLCPP_ERROR(logger_, "CAN sender inhibited: reason=%s. "
                     "IDLE requested but unconfirmed; explicit Rest/recovery required.",
                     commands_.fault_reason_name());
    }
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) return;
    if (idle_shutdown_) idle_shutdown_->observe(frame);
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
            if (!(frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) &&
                (frame.can_id & 0x1f) == Get_Encoder_Estimates_msg_t::cmd_id &&
                frame.can_dlc == Get_Encoder_Estimates_msg_t::msg_length &&
                std::isfinite(axis.pos_estimate_) && std::isfinite(axis.vel_estimate_)) {
                const auto steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                axis.encoder_timing_->receive(can_intf_.last_receive_timestamp_ns(), wall, steady);
            }
        }
    }
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };  

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                // Position and its derivative must share the motor-to-joint sign.
                constexpr double TURNS_TO_RAD = 2.0 * M_PI;
                constexpr double GR = 9.0; // Gear ratio
                pos_estimate_ = -(msg.Pos_Estimate * TURNS_TO_RAD / GR);
                vel_estimate_ = -(msg.Vel_Estimate * TURNS_TO_RAD / GR);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)

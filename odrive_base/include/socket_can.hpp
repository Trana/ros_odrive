#ifndef SOCKET_CAN_HPP
#define SOCKET_CAN_HPP

#include "epoll_event_loop.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>

using FrameProcessor = std::function<void(const can_frame&)>;

class SocketCanIntf {
public:
    bool init(const std::string& interface, EpollEventLoop* event_loop, FrameProcessor frame_processor);
    void deinit();
    bool send_can_frame(const can_frame& frame);

    bool read_nonblocking();
    // Configure before init; metadata belongs to the current frame callback only.
    bool collect_receive_timestamps = false;
    int64_t last_receive_timestamp_ns() const { return last_receive_timestamp_ns_; }

private:
    int64_t last_receive_timestamp_ns_ = 0;
    std::string interface_;
    int socket_id_ = -1;
    EpollEventLoop* event_loop_ = nullptr;
    EpollEventLoop::EvtId socket_evt_id_;
    FrameProcessor frame_processor_;
    bool broken_ = false;
    std::chrono::steady_clock::time_point last_send_error_log_;
    std::size_t suppressed_send_errors_ = 0;

    void on_socket_event(uint32_t mask);
    void log_send_error(int error_number);
    void process_can_frame(const can_frame& frame) { frame_processor_(frame); }
};

#endif // SOCKET_CAN_HPP

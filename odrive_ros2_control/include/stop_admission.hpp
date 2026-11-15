#pragma once

#include <atomic>
#include <cstdint>

namespace odrive_ros2_control {

// A later activation may retire a completed Stop. It cannot erase a concurrent
// request, including one already waiting for the transport mutex.
class StopAdmission {
public:
    using Token = uint64_t;
    class Request {
    public:
        explicit Request(StopAdmission& owner) : owner_(owner) {
            owner_.pending_.fetch_add(1, std::memory_order_acq_rel);
            owner_.sequence_.fetch_add(1, std::memory_order_acq_rel);
            owner_.latched_.store(true, std::memory_order_release);
        }
        Request(const Request&) = delete;
        Request& operator=(const Request&) = delete;
        ~Request() { owner_.pending_.fetch_sub(1, std::memory_order_acq_rel); }
    private:
        StopAdmission& owner_;
    };
    Request request() { return Request(*this); }
    Token token() const { return sequence_.load(std::memory_order_acquire); }
    bool can_admit(Token token) const {
        return pending_.load(std::memory_order_acquire) == 0 && token == this->token();
    }
    bool admit(Token token) {
        if (!can_admit(token)) return false;
        latched_.store(false, std::memory_order_release);
        if (allowed(token)) return true;
        latched_.store(true, std::memory_order_release);
        return false;
    }
    bool allowed(Token token) const { return can_admit(token) && !requested(); }
    bool requested() const {
        return pending_.load(std::memory_order_acquire) != 0 || latched_.load(std::memory_order_acquire);
    }

private:
    std::atomic<uint64_t> sequence_{0};
    std::atomic<unsigned> pending_{0};
    std::atomic<bool> latched_{true};
};
} // namespace odrive_ros2_control

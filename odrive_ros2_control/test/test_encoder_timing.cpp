#include "encoder_timing.hpp"
#include "receive_timestamp.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
using odrive_ros2_control::EncoderTiming;
void check(bool v) {
    if (!v)
        throw std::runtime_error("encoder timing assertion");
}
int main() {
    EncoderTiming t;
    check(std::isnan(t.receipt_monotonic_seconds()));
    constexpr int64_t wall = 10000000000, steady = 1000000000;
    t.sample(wall, steady);
    check(t.never_received_reads == 1);
    t.receive(wall - 3000000, wall, steady);
    check(std::abs(t.receipt_monotonic_seconds() - 0.997) < 1e-12);
    t.sample(wall + 2000000, steady + 2000000);
    check(t.queue_delay.max_ns == 3000000 && t.age_at_read.max_ns == 5000000);
    std::ostringstream output;
    t.json(output, 11);
    check(output.str().find("\"node_id\":11") != std::string::npos);
    check(t.frames == 0 && t.age_at_read.count == 0);
    // Receipt survives reporting boundaries and ages without fresh packets.
    t.sample(wall + 10000000, steady + 10000000);
    check(t.age_at_read.max_ns == 13000000);
    // Clock discontinuity invalidates cached age until new valid receipt.
    t.sample(wall + 30000000, steady + 11000000);
    check(t.invalid_clock == 1 && t.unavailable_age_reads == 1);
    check(std::isnan(t.receipt_monotonic_seconds()));
    t.receive(0, wall, steady);
    check(t.missing_stamp == 1);
    t.sample(wall, steady);
    check(t.unavailable_age_reads == 2);
    t.receive(wall + 1, wall, steady);
    check(t.invalid_clock == 2);
    t.receive(wall - 1000, wall, steady);
    t.sample(wall, steady);
    check(t.age_at_read.count == 2);
    t.reset_receipt();
    check(std::isnan(t.receipt_monotonic_seconds()));
    t.sample(wall, steady);
    check(t.never_received_reads == 1);

    alignas(cmsghdr) char buffer[CMSG_SPACE(sizeof(timespec))]{};
    msghdr message{};
    check(receive_timestamp_ns(message) == 0);
    message.msg_control = buffer;
    message.msg_controllen = sizeof(buffer);
    auto* c = CMSG_FIRSTHDR(&message);
    c->cmsg_level = SOL_SOCKET;
    c->cmsg_type = SO_TIMESTAMPNS;
    c->cmsg_len = CMSG_LEN(sizeof(timespec));
    timespec stamp{10, 123};
    std::memcpy(CMSG_DATA(c), &stamp, sizeof(stamp));
    check(receive_timestamp_ns(message) == 10000000123);
    message.msg_flags = MSG_CTRUNC;
    check(receive_timestamp_ns(message) == 0);
    message.msg_flags = 0;
    c->cmsg_len = CMSG_LEN(sizeof(timespec)) - 1;
    check(receive_timestamp_ns(message) == 0);
    c->cmsg_len = CMSG_LEN(sizeof(timespec));
    stamp.tv_nsec = 1000000000;
    std::memcpy(CMSG_DATA(c), &stamp, sizeof(stamp));
    check(receive_timestamp_ns(message) == 0);
    std::cout << "Encoder queue/age and ancillary timestamp tests passed\n";
}

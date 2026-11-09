#pragma once
#include <cstdint>
#include <cstring>
#include <ctime>
#include <sys/socket.h>

// Zero means unavailable, never substitute userspace receipt for kernel arrival.
inline int64_t receive_timestamp_ns(msghdr& message) {
    if (message.msg_flags & MSG_CTRUNC)
        return 0;
    for (auto* c = CMSG_FIRSTHDR(&message); c; c = CMSG_NXTHDR(&message, c)) {
        if (c->cmsg_level != SOL_SOCKET || c->cmsg_type != SO_TIMESTAMPNS || c->cmsg_len < CMSG_LEN(sizeof(timespec)))
            continue;
        timespec stamp{};
        std::memcpy(&stamp, CMSG_DATA(c), sizeof(stamp));
        if (stamp.tv_sec <= 0 || stamp.tv_sec > INT64_MAX / 1000000000 - 1 || stamp.tv_nsec < 0
            || stamp.tv_nsec >= 1000000000)
            return 0;
        return static_cast<int64_t>(stamp.tv_sec) * 1000000000 + stamp.tv_nsec;
    }
    return 0;
}

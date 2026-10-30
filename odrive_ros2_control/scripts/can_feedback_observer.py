"""Bounded receive-only observer; kernel timestamp age, never CAN sends."""
import socket
import struct
import time

from can_diagnostic_guard import DiagnosticAbort


class FeedbackObserver:
    def __init__(self, interface='can0'):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.timestamp_option = getattr(socket, 'SO_TIMESTAMPNS', 35)
        self.drop_option = getattr(socket, 'SO_RXQ_OVFL', 40)
        # Exclude extended/RTR frames. CAN_ERR_FLAG must not be in the normal
        # filter mask: SocketCAN routes that registration to error subscriptions.
        filters = b''.join(struct.pack('=II', cmd, 0xC000001F) for cmd in (1, 9))
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        # Receive all CAN error classes; any error frame remains fatal.
        self.sock.setsockopt(socket.SOL_CAN_RAW, getattr(socket, 'CAN_RAW_ERR_FILTER', 2),
                             struct.pack('=I', 0x1FFFFFFF))
        self.sock.setsockopt(socket.SOL_SOCKET, self.timestamp_option, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, self.drop_option, 1)
        self.sock.bind((interface,))
        self.sock.setblocking(False)
        self.offset = time.time() - time.monotonic()
        self.states = {}

    def drain(self, guard):
        for _ in range(4096):
            try:
                payload, ancillary, flags, _ = self.sock.recvmsg(16, 256)
            except BlockingIOError:
                return
            if flags & (socket.MSG_CTRUNC | socket.MSG_TRUNC) or len(payload) != 16:
                raise DiagnosticAbort('observer truncated record')
            ident, size, data = struct.unpack('=IB3x8s', payload)
            if ident & 0x20000000:
                # Permit only an RX-overflow notification; detailed interface
                # counters still enforce total/burst budgets. All other CAN
                # protocol/controller errors, including bus-off, remain fatal.
                if (ident & 0x1FFFFFFF) != 0x4 or size != 8 or data[1] != 0x1:
                    raise DiagnosticAbort('CAN error frame: ' + hex(ident))
                continue
            if ident & 0xC0000000 or size != 8:
                raise DiagnosticAbort('invalid feedback frame')
            stamp = None
            for level, kind, value in ancillary:
                if level != socket.SOL_SOCKET:
                    continue
                if kind == self.timestamp_option and len(value) >= struct.calcsize('@ll'):
                    sec, ns = struct.unpack_from('@ll', value)
                    if sec > 0 and 0 <= ns < 1_000_000_000:
                        stamp = sec + ns / 1e9
                if kind == self.drop_option and len(value) >= 4:
                    if struct.unpack_from('=I', value)[0]:
                        raise DiagnosticAbort('observer socket dropped frames')
            wall, mono = time.time(), time.monotonic()
            if abs((wall - mono) - self.offset) > .005:
                raise DiagnosticAbort('wall/monotonic clock mapping changed')
            if stamp is None or stamp > wall:
                raise DiagnosticAbort('missing/future kernel timestamp')
            received = mono - (wall - stamp)
            node, cmd = ident >> 5, ident & 31
            if node not in guard.nodes:
                continue
            if cmd == 9:
                guard.observe(node, received)
            elif cmd == 1:
                self.states[node] = (received, data[4], int.from_bytes(data[:4], 'little'))
        raise DiagnosticAbort('observer drain limit exceeded')

    def close(self):
        self.sock.close()

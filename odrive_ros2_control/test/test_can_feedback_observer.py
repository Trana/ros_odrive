import socket
import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from can_feedback_observer import FeedbackObserver
from can_diagnostic_guard import DiagnosticAbort, DiagnosticGuard


class ObserverTests(unittest.TestCase):
    def test_socket_filter_excludes_error_flag(self):
        with patch('can_feedback_observer.socket.socket') as factory:
            observer = FeedbackObserver()
        filters = [call.args[2] for call in factory.return_value.setsockopt.call_args_list
                   if call.args[:2] == (socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER)][0]
        self.assertEqual(struct.unpack('=IIII', filters), (1, 0xC000001F, 9, 0xC000001F))
        observer.close()

    def observer(self, ident=11 * 32 + 9, data=b'\0' * 8, stamp=99.99, extra=()):
        observer = FeedbackObserver.__new__(FeedbackObserver)
        observer.timestamp_option, observer.drop_option = 35, 40
        observer.offset, observer.states = 90, {}
        class FakeSocket:
            def __init__(self): self.done = False
            def recvmsg(self, *_):
                if self.done: raise BlockingIOError()
                self.done = True
                anc = list(extra)
                if stamp is not None:
                    anc.append((socket.SOL_SOCKET, 35, struct.pack('@ll', int(stamp), int((stamp % 1)*1e9))))
                return struct.pack('=IB3x8s', ident, 8, data), anc, 0, None
        observer.sock = FakeSocket()
        return observer

    def drain(self, observer):
        g = DiagnosticGuard([11], {})
        with patch('can_feedback_observer.time.time', return_value=100), patch(
                'can_feedback_observer.time.monotonic', return_value=10):
            observer.drain(g)
        return g

    def test_old_kernel_stamp_not_dequeue_time(self):
        g = self.drain(self.observer(stamp=99.8))
        self.assertAlmostEqual(g.feedback[11], 9.8)

    def test_missing_timestamp(self):
        with self.assertRaises(DiagnosticAbort): self.drain(self.observer(stamp=None))

    def test_socket_drop(self):
        with self.assertRaises(DiagnosticAbort):
            self.drain(self.observer(extra=[(socket.SOL_SOCKET, 40, struct.pack('=I', 1))]))

    def test_bus_off(self):
        with self.assertRaises(DiagnosticAbort): self.drain(self.observer(ident=0x20000040))

    def test_overflow_notification_delegated_to_counters(self):
        self.drain(self.observer(ident=0x20000004, data=b'\0\1'+b'\0'*6))

    def test_heartbeat(self):
        obs = self.observer(ident=11*32+1, data=b'\0'*4+b'\10'+b'\0'*3)
        self.drain(obs)
        self.assertEqual(obs.states[11][1:], (8, 0))


if __name__ == '__main__': unittest.main()

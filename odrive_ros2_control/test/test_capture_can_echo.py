import importlib.util
from pathlib import Path
import socket
import struct
import unittest

spec = importlib.util.spec_from_file_location("capture_can_echo", Path(__file__).parents[1] / "scripts/capture_can_echo.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def frame(command, flags=0, length=8):
    return struct.pack("=IB3x8s", (22 << 5) | command | flags, length, b"12345678")


class CaptureFrameTest(unittest.TestCase):
    def test_default_accepts_local_position_only(self):
        result = module.classify_frame(frame(0x0C), socket.MSG_DONTROUTE)
        self.assertEqual(result['node_id'], 22)
        self.assertEqual(result['frame_type'], 'local_position_echo')
        self.assertIsNone(module.classify_frame(frame(0x0C), 0))
        self.assertIsNone(module.classify_frame(frame(0x09), 0))

    def test_opt_in_feedback_excludes_local_echo(self):
        result = module.classify_frame(frame(0x09), 0, True)
        self.assertEqual(result['frame_type'], 'encoder_feedback')
        self.assertIsNone(module.classify_frame(frame(0x09), socket.MSG_DONTROUTE, True))

    def test_invalid_and_unrelated_frames(self):
        for payload in (b'bad', frame(1), frame(9, length=4), frame(9, 0x80000000),
                        frame(9, 0x40000000), frame(9, 0x20000000)):
            self.assertIsNone(module.classify_frame(payload, 0, True))

    def test_clock_bracket_is_ordered(self):
        pair = module.clock_pair()
        self.assertLessEqual(pair['monotonic_before_ns'], pair['monotonic_after_ns'])
        self.assertGreater(pair['unix_ns'], 0)


if __name__ == '__main__':
    unittest.main()

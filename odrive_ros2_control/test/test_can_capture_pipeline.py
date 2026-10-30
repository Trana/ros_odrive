"""Exercise recorder CLI and analyzer without opening any real socket."""
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import socket
import struct
import unittest
from unittest.mock import patch


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / f'scripts/{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


recorder = load('capture_can_echo')
analyzer = load('analyze_can_timing')


class Clock:
    tick = 0

    def monotonic(self):
        self.tick += 50_000_000
        return self.tick / 1e9

    def monotonic_ns(self):
        return self.tick

    def time_ns(self):
        return 1_000_000_000_000 + self.tick


class ReceiveOnlySocket:
    """Deliberately has no send method or interface-control method."""
    def __init__(self, clock, truncated=False, drops=0):
        self.clock, self.truncated, self.drops = clock, truncated, drops
        self.index = 0
        self.options = []

    def __enter__(self): return self
    def __exit__(self, *args): pass
    def setsockopt(self, *args): self.options.append(args)
    def bind(self, address): self.address = address
    def settimeout(self, seconds): self.timeout = seconds

    def recvmsg(self, *args):
        if self.index >= 2:
            raise socket.timeout()
        command = (0x0C, 0x09)[self.index]
        flags = socket.MSG_DONTROUTE if self.index == 0 else 0
        if self.truncated: flags |= socket.MSG_CTRUNC
        self.index += 1
        sec, ns = divmod(self.clock.time_ns(), 1_000_000_000)
        ancillary = [(socket.SOL_SOCKET, getattr(socket, 'SO_TIMESTAMPNS', 35), struct.pack('@ll', sec, ns)),
                     (socket.SOL_SOCKET, getattr(socket, 'SO_RXQ_OVFL', 40), struct.pack('=I', self.drops))]
        payload = struct.pack('=IB3x8s', (22 << 5) | command, 8, struct.pack('<ff', 1.0, 2.0))
        return payload, ancillary, flags, None


def run_capture(combined=True, **kwargs):
    clock = Clock()
    observer = ReceiveOnlySocket(clock, **kwargs)
    args = ['capture_can_echo', '--interface', 'test-only', '--seconds', '1']
    if combined: args.append('--include-encoder-feedback')
    output = io.StringIO()
    with patch.object(recorder.socket, 'socket', return_value=observer), \
         patch.object(recorder.time, 'monotonic', clock.monotonic), \
         patch.object(recorder.time, 'monotonic_ns', clock.monotonic_ns), \
         patch.object(recorder.time, 'time_ns', clock.time_ns), \
         patch('sys.argv', args), redirect_stdout(output):
        recorder.main()
    return json.loads(output.getvalue()), observer


class CapturePipelineTest(unittest.TestCase):
    def test_combined_round_trip(self):
        data, observer = run_capture()
        self.assertEqual(observer.address, ('test-only',))
        self.assertEqual(len(observer.options[0][2]), 16)  # Two CAN filters.
        self.assertEqual(data['local_position_frames'], 1)
        self.assertEqual(data['encoder_feedback_frames'], 1)
        self.assertTrue(data['hostname'])
        self.assertEqual(len(data['recorder_sha256']), 64)
        self.assertIn('boot_id', data)
        self.assertTrue(analyzer.analyze(data)['ready_for_cross_stream_analysis'])

    def test_default_retains_echo_only_contract(self):
        data, observer = run_capture(combined=False)
        self.assertEqual(data['schema'], 'can-echo-observer.v1')
        self.assertEqual(len(observer.options[0][2]), 8)
        self.assertEqual(len(data['records']), 1)
        self.assertEqual(data['encoder_feedback_frames'], 0)
        self.assertEqual(data['clock_pairs'], [])

    def test_truncation_reaches_quality_gate(self):
        data, _ = run_capture(truncated=True)
        self.assertEqual(data['missing_timestamps'], 2)
        self.assertEqual(data['truncated_ancillary'], 2)
        self.assertFalse(analyzer.analyze(data)['ready_for_cross_stream_analysis'])

    def test_socket_drop_count_reaches_quality_gate(self):
        data, _ = run_capture(drops=3)
        self.assertEqual(data['socket_drops_reported'], 3)
        self.assertFalse(analyzer.analyze(data)['ready_for_cross_stream_analysis'])


if __name__ == '__main__': unittest.main()

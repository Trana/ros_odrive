import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location('analyze_can_timing', Path(__file__).parents[1] / 'scripts/analyze_can_timing.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def capture():
    return dict(schema='can-timing-observer.v1', missing_timestamps=0,
                truncated_ancillary=0, socket_drops_reported=0, capture_limit_drops=0,
                clock_pairs=[dict(monotonic_before_ns=t, monotonic_after_ns=t+100, unix_ns=t+1000000)
                             for t in (100, 1000000100)], records=[
        dict(node_id=22, frame_type=kind, observer_monotonic_ns=100+i,
             kernel_receive_ns=100+i, payload_hex=struct.pack('<ff', 1.5, 2.0).hex())
        for i, kind in enumerate(('local_position_echo', 'encoder_feedback'))])


class TimingAnalysisTest(unittest.TestCase):
    def test_units_and_stream_separation(self):
        result = module.analyze(capture())
        self.assertTrue(result['ready_for_cross_stream_analysis'])
        self.assertEqual(result['streams']['22/encoder_feedback']['velocity_range_rev_s'], [2.0, 2.0])
        self.assertIsNone(result['streams']['22/local_position_echo']['velocity_range_rev_s'])

    def test_losses_prevent_readiness(self):
        data = capture()
        data['socket_drops_reported'] = 2
        self.assertFalse(module.analyze(data)['ready_for_cross_stream_analysis'])

    def test_clock_step_prevents_readiness(self):
        data = capture()
        data['clock_pairs'][1]['unix_ns'] += 10000000
        self.assertIn('clock_alignment_uncertain', module.analyze(data)['quality_issues'])

    def test_missing_stamp_not_silently_accepted(self):
        data = capture()
        data['records'][0]['kernel_receive_ns'] = None
        self.assertFalse(module.analyze(data)['ready_for_cross_stream_analysis'])

    def test_invalid_data_rejected(self):
        for change in ('schema', 'nan', 'regression'):
            data = capture()
            if change == 'schema': data['schema'] = 'can-echo-observer.v1'
            if change == 'nan': data['records'][0]['payload_hex'] = struct.pack('<ff', float('nan'), 0).hex()
            if change == 'regression': data['records'][1]['kernel_receive_ns'] = 0
            with self.assertRaises(ValueError): module.analyze(data)

    def test_empty_capture_is_not_ready(self):
        data = capture()
        data['records'] = []
        self.assertFalse(module.analyze(data)['ready_for_cross_stream_analysis'])

    def test_different_nodes_do_not_form_a_pair(self):
        data = capture()
        data['records'][1]['node_id'] = 23
        result = module.analyze(data)
        self.assertEqual(result['unpaired_nodes'], [22, 23])
        self.assertFalse(result['ready_for_cross_stream_analysis'])


if __name__ == '__main__': unittest.main()

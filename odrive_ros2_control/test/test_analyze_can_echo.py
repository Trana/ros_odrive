import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "analyze_can_echo", Path(__file__).parents[1] / "scripts/analyze_can_echo.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def capture(nodes, stamps=None):
    if stamps is None:
        stamps = [i * 1_000_000 for i in range(len(nodes))]
    return {"schema": "can-echo-observer.v1", "records": [
        {"node_id": node, "kernel_receive_ns": stamp}
        for node, stamp in zip(nodes, stamps)]}


class EchoAnalysisTest(unittest.TestCase):
    def test_complete_and_partial(self):
        result = module.analyze(capture([1, 2, 3, 1, 2, 3, 1]), (1, 2, 3))
        self.assertEqual(result["complete_rounds"], 2)
        self.assertEqual(result["round_echo_span_ms"]["mean"], 2)
        self.assertEqual(result["partial_round_frames"], 1)
        self.assertEqual(result["per_node"]["1"]["echo_gap_ms"]["max"], 3)

    def test_resync(self):
        result = module.analyze(capture([3, 1, 2, 1, 2, 3]), (1, 2, 3))
        self.assertEqual(result["unmatched_frames"], 1)
        self.assertEqual(result["partial_round_frames"], 2)
        self.assertEqual(result["complete_rounds"], 1)

    def test_window(self):
        result = module.analyze(capture([1, 2, 3, 1, 2, 3]), (1, 2, 3), 1_000_000, 5_000_000)
        self.assertEqual(result["selected_frames"], 4)
        self.assertEqual(result["complete_rounds"], 0)
        self.assertIsNone(result["round_echo_span_ms"]["mean"])

    def test_missing_does_not_bridge_round(self):
        result = module.analyze(capture([1, 2, 3], [0, None, 2_000_000]), (1, 2, 3))
        self.assertEqual(result["complete_rounds"], 0)
        self.assertEqual(result["missing_timestamp_records_full_capture"], 1)

    def test_regression_rejected(self):
        with self.assertRaises(ValueError):
            module.analyze(capture([1, 2], [20, 10]), (1, 2))

    def test_empty(self):
        self.assertEqual(module.analyze(capture([]))["selected_frames"], 0)

    def test_invalid_inputs(self):
        for order in [(), (1, 1)]:
            with self.assertRaises(ValueError):
                module.analyze(capture([]), order)
        with self.assertRaises(ValueError):
            module.analyze(capture([]), start_ns=3, end_ns=3)
        with self.assertRaises(ValueError):
            module.analyze({"schema": "wrong"})

    def test_quality_preserved(self):
        data = capture([1])
        data["socket_drops_reported"] = 4
        self.assertEqual(module.analyze(data)["capture_quality_full_capture"]["socket_drops_reported"], 4)


if __name__ == "__main__":
    unittest.main()

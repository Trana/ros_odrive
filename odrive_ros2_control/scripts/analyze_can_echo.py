#!/usr/bin/env python3
"""Offline analysis of capture_can_echo.py JSON; never accesses CAN or SSH.

Ordered echo groups are heuristic rounds, NOT policy generations, hardware
timestamps or motor acknowledgements. No inference about RX errors is possible
from this position-echo-only capture.
"""
import argparse
import json
import math
from pathlib import Path

DEFAULT_ORDER = (13, 12, 11, 23, 22, 21, 33, 32, 31, 43, 42, 41)


def distribution(values):
    if not values:
        return {"count": 0, "mean": None, "p95": None, "max": None}
    ordered = sorted(values)
    return {"count": len(values), "mean": sum(values) / len(values),
            "p95": ordered[math.ceil(0.95 * len(values)) - 1],
            "max": ordered[-1]}


def analyze(data, order=DEFAULT_ORDER, start_ns=None, end_ns=None):
    if data.get("schema") != "can-echo-observer.v1":
        raise ValueError("unsupported echo schema")
    if not order or len(set(order)) != len(order):
        raise ValueError("node order must be nonempty and unique")
    if start_ns is not None and end_ns is not None and start_ns >= end_ns:
        raise ValueError("end must be after start")
    spans, records = [], []
    pending = []
    unmatched = partial_frames = missing = 0
    previous = None
    for record in data["records"]:
        stamp = record["kernel_receive_ns"]
        if stamp is None:
            missing += 1
            partial_frames += len(pending)
            pending = []  # Do not silently bridge an unknown timestamp.
            continue
        if not isinstance(stamp, int) or isinstance(stamp, bool):
            raise ValueError("timestamps must be integer nanoseconds")
        if previous is not None and stamp < previous:
            raise ValueError("timestamp regression: cannot trust interval analysis")
        previous = stamp
        if start_ns is not None and stamp < start_ns:
            continue
        if end_ns is not None and stamp >= end_ns:
            continue
        records.append(record)
        node = record["node_id"]
        if node != order[len(pending)]:
            partial_frames += len(pending)
            pending = []
            if node != order[0]:
                unmatched += 1
                continue
        pending.append(stamp)
        if len(pending) == len(order):
            spans.append((pending[-1] - pending[0]) / 1e6)
            pending = []
    partial_frames += len(pending)
    per_node = {}
    for node in order:
        stamps = [r["kernel_receive_ns"] for r in records if r["node_id"] == node]
        per_node[str(node)] = {
            "frames": len(stamps),
            "echo_gap_ms": distribution([(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]),
        }
    quality = {key: data.get(key) for key in (
        "missing_timestamps", "truncated_ancillary", "socket_drops_reported", "capture_limit_drops")}
    return {
        "schema": "can-echo-analysis.v1",
        "window": {"start_inclusive_ns": start_ns, "end_exclusive_ns": end_ns},
        "selected_frames": len(records), "complete_rounds": len(spans),
        "round_echo_span_ms": distribution(spans),
        "partial_round_frames": partial_frames, "unmatched_frames": unmatched,
        "missing_timestamp_records_full_capture": missing,
        "capture_quality_full_capture": quality,
        "per_node": per_node,
        "limitations": [
            "No CAN error counters or encoder feedback in this capture.",
            "Echo gaps are not encoder gaps or joint-data age.",
            "Software echo timing is not exact on-wire or motor-application timing.",
            "Sequence resynchronization cannot establish policy-action generations.",
            "Partial/unmatched records may be window boundaries, not lost frames.",
            "Aborted captures are not successful reliability trials.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--start-ns", type=int, help="inclusive Unix nanoseconds")
    parser.add_argument("--end-ns", type=int, help="exclusive Unix nanoseconds")
    parser.add_argument("--nodes", default=",".join(map(str, DEFAULT_ORDER)))
    args = parser.parse_args()
    try:
        result = analyze(json.loads(args.capture.read_text()),
                         tuple(int(n) for n in args.nodes.split(",")),
                         args.start_ns, args.end_ns)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()

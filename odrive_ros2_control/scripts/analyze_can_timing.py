#!/usr/bin/env python3
"""Read-only quality analysis of combined CAN captures. No CAN or ROS access."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import struct


def integer(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f'{name} must be a nonnegative integer')
    return value


def analyze(data, clock_tolerance_ms=1.0):
    if data.get('schema') != 'can-timing-observer.v1':
        raise ValueError('Expected combined can-timing-observer.v1 schema')
    if not math.isfinite(clock_tolerance_ms) or clock_tolerance_ms <= 0:
        raise ValueError('Clock tolerance must be finite and positive')
    issues = []
    quality = {key: integer(data[key], key) for key in (
        'missing_timestamps', 'truncated_ancillary', 'socket_drops_reported', 'capture_limit_drops')}
    issues.extend(key for key, value in quality.items() if value)
    offsets, widths = [], []
    last_clock = None
    for pair in data['clock_pairs']:
        before = integer(pair['monotonic_before_ns'], 'clock before')
        after = integer(pair['monotonic_after_ns'], 'clock after')
        wall = integer(pair['unix_ns'], 'wall clock')
        if after < before or (last_clock is not None and before < last_clock):
            raise ValueError('Unordered clock brackets')
        last_clock = after
        offsets.append(wall - (before + after) / 2)
        widths.append(after - before)
    span_ms = (max(offsets) - min(offsets)) / 1e6 if offsets else None
    max_bracket_ms = max(widths) / 1e6 if widths else None
    if len(offsets) < 2:
        issues.append('insufficient_clock_pairs')
    elif max_bracket_ms > clock_tolerance_ms or span_ms > clock_tolerance_ms:
        issues.append('clock_alignment_uncertain')
    streams = defaultdict(lambda: {'positions': [], 'velocities': [], 'gaps': [], 'last_stamp': None, 'missing': 0})
    last_observed = last_kernel = None
    for record in data['records']:
        node = integer(record['node_id'], 'node_id')
        kind = record['frame_type']
        if node > 63 or kind not in ('local_position_echo', 'encoder_feedback'):
            raise ValueError('Invalid node or frame type')
        observer = integer(record['observer_monotonic_ns'], 'observer timestamp')
        if last_observed is not None and observer < last_observed:
            raise ValueError('Observer timestamp regression')
        last_observed = observer
        payload = bytes.fromhex(record['payload_hex'])
        if len(payload) != 8:
            raise ValueError('Expected eight-byte CAN payload')
        position = struct.unpack_from('<f', payload)[0]
        velocity = struct.unpack_from('<f', payload, 4)[0] if kind == 'encoder_feedback' else None
        if not math.isfinite(position) or (velocity is not None and not math.isfinite(velocity)):
            raise ValueError('Nonfinite motor value')
        stream = streams[(node, kind)]
        stream['positions'].append(position)
        if velocity is not None:
            stream['velocities'].append(velocity)
        stamp = record['kernel_receive_ns']
        if stamp is None:
            stream['missing'] += 1
            stream['last_stamp'] = None  # Never bridge an unknown timestamp.
            continue
        stamp = integer(stamp, 'kernel timestamp')
        if last_kernel is not None and stamp < last_kernel:
            raise ValueError('Kernel timestamp regression; select a clean clock epoch')
        last_kernel = stamp
        if stream['last_stamp'] is not None:
            stream['gaps'].append((stamp - stream['last_stamp']) / 1e6)
        stream['last_stamp'] = stamp
    output = {}
    for (node, kind), stream in sorted(streams.items()):
        gaps, positions = stream['gaps'], stream['positions']
        output[f'{node}/{kind}'] = {
            'frames': len(positions), 'missing_timestamps': stream['missing'],
            'gap_count': len(gaps), 'gap_mean_ms': sum(gaps)/len(gaps) if gaps else None,
            'gap_max_ms': max(gaps) if gaps else None,
            'position_range_rev': [min(positions), max(positions)],
            'velocity_range_rev_s': [min(stream['velocities']), max(stream['velocities'])] if stream['velocities'] else None,
        }
    if any(stream['missing'] for stream in streams.values()) and 'missing_timestamps' not in issues:
        issues.append('unreported_missing_timestamps')
    kinds = {kind for _, kind in streams}
    both = kinds == {'local_position_echo', 'encoder_feedback'}
    nodes = {node for node, _ in streams}
    unpaired = sorted(node for node in nodes if any((node, kind) not in streams
                      for kind in ('local_position_echo', 'encoder_feedback')))
    if unpaired:
        issues.append('unpaired_node_streams')
    return {
        'schema': 'can-timing-quality.v1', 'streams': output, 'capture_quality': quality,
        'capture_identity': {key: data.get(key) for key in ('hostname', 'boot_id', 'recorder_sha256')},
        'clock_pairs': len(offsets), 'clock_offset_span_ms': span_ms,
        'max_clock_bracket_ms': max_bracket_ms, 'clock_tolerance_ms': clock_tolerance_ms,
        'quality_issues': issues, 'has_both_frame_types': both,
        'unpaired_nodes': unpaired,
        'ready_for_cross_stream_analysis': both and not issues,
        'limits': 'Readiness is a data-quality check, not a motion verdict. Software receipt/echo times are not encoder acquisition or motor acknowledgement. Motor revolutions are not joint radians. No unique action matching, filter identification, or latency estimate is performed.',
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(json.loads(args.capture.read_text())), indent=2, allow_nan=False))

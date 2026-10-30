#!/usr/bin/env python3
"""Bounded, receive-only capture of ODrive position echoes and optional feedback.

Never transmits. Echo receipt is not a hardware timestamp or motor acknowledgement.
Run alongside an authorized test; write stdout to its evidence directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import struct
import time


def classify_frame(payload, flags, include_encoder_feedback=False):
    """Accept classic standard frames only; never generate CAN traffic."""
    if len(payload) != 16:
        return None
    ident, length, data = struct.unpack("=IB3x8s", payload)
    if ident & 0xE0000000 or length != 8:
        return None
    command = ident & 0x1F
    local = bool(flags & socket.MSG_DONTROUTE)
    if command == 0x0C and local:
        kind = "local_position_echo"
    elif command == 0x09 and not local and include_encoder_feedback:
        kind = "encoder_feedback"
    else:
        return None
    return {"node_id": ident >> 5, "frame_type": kind, "payload_hex": data.hex()}


def clock_pair():
    """Bracket wall time on the observer host for clock-domain alignment."""
    before = time.monotonic_ns()
    wall = time.time_ns()
    return {"monotonic_before_ns": before, "unix_ns": wall,
            "monotonic_after_ns": time.monotonic_ns()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--seconds", type=int, default=20, choices=range(1, 61))
    parser.add_argument("--include-encoder-feedback", action="store_true",
                        help="Record remote encoder estimates as well, using can-timing-observer.v1")
    args = parser.parse_args()
    # Hash before the measurement interval. Record clock-domain provenance.
    identity = {"hostname": socket.gethostname(),
                "recorder_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    try:
        identity["boot_id"] = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        identity["boot_id"] = None
    timestamp_option = getattr(socket, "SO_TIMESTAMPNS", 35)
    drop_option = getattr(socket, "SO_RXQ_OVFL", 40)
    records = []
    total = encoder_total = missing = truncated = discarded = dropped = 0
    started_wall = time.time_ns()
    started = time.monotonic()
    clock_pairs = [clock_pair()] if args.include_encoder_feedback else []
    next_clock = started + 1.0
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as observer:
        # Standard non-RTR Set_Input_Pos (command 0x0c), any node.
        filters = struct.pack("=II", 0x0C, 0xC000001F)
        if args.include_encoder_feedback:
            filters += struct.pack("=II", 0x09, 0xC000001F)
        observer.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        observer.setsockopt(socket.SOL_SOCKET, timestamp_option, 1)
        observer.setsockopt(socket.SOL_SOCKET, drop_option, 1)
        observer.bind((args.interface,))
        while time.monotonic() - started < args.seconds:
            if args.include_encoder_feedback and time.monotonic() >= next_clock:
                clock_pairs.append(clock_pair())
                next_clock = time.monotonic() + 1.0
            observer.settimeout(max(0.001, min(0.2, args.seconds - (time.monotonic() - started))))
            try:
                payload, ancillary, flags, _ = observer.recvmsg(16, 256)
            except socket.timeout:
                continue
            record = classify_frame(payload, flags, args.include_encoder_feedback)
            if record is None:
                continue
            if record["frame_type"] == "local_position_echo":
                total += 1
            else:
                encoder_total += 1
            stamp = None
            if flags & socket.MSG_CTRUNC:
                truncated += 1
            else:
                for level, kind, value in ancillary:
                    if level != socket.SOL_SOCKET:
                        continue
                    if kind == timestamp_option and len(value) >= struct.calcsize("@ll"):
                        sec, ns = struct.unpack_from("@ll", value)
                        if sec > 0 and 0 <= ns < 1_000_000_000:
                            stamp = sec * 1_000_000_000 + ns
                    if kind == drop_option and len(value) >= 4:
                        dropped = struct.unpack_from("=I", value)[0]
            missing += stamp is None
            if len(records) < 40000:
                records.append({**record, "kernel_receive_ns": stamp,
                                "observer_monotonic_ns": time.monotonic_ns()})
            else:
                discarded += 1
    if args.include_encoder_feedback:
        clock_pairs.append(clock_pair())
    print(json.dumps({**identity, "schema": "can-timing-observer.v1" if args.include_encoder_feedback else "can-echo-observer.v1", "interface": args.interface,
                      "start_unix_ns": started_wall, "elapsed_s": time.monotonic() - started,
                      "local_position_frames": total, "missing_timestamps": missing,
                      "encoder_feedback_frames": encoder_total, "clock_pairs": clock_pairs,
                      "truncated_ancillary": truncated, "socket_drops_reported": dropped,
                      "capture_limit_drops": discarded, "records": records}))


if __name__ == "__main__":
    main()

"""
ODrive CAN helper (organized/refactored)

Supports:
- Check firmware/hardware version via CANSimple Get_Version
- SDO read/write (config + status)
- Save configuration (SDO)
- Reboot (SDO)
- Set control mode (CANSimple Set_Controller_Mode)
- Easy one-line status printing per node

Requirements:
- python-can
- flat_endpoints.json (your generated endpoint map) in same folder
"""

import json
import struct
import time
import can
from dataclasses import dataclass
from typing import Dict, Any, List, Optional


# ----------------------------
# User config
# ----------------------------
CAN_IFACE = "can0"
ENDPOINTS_JSON = "flat_endpoints.json"

NODE_IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]
# NODE_IDS = [43]

# Per-node values to write (example: gains)
CONFIG_TO_WRITE: Dict[int, Dict[str, Any]] = {
    nid: {
        # "axis0.pos_vel_mapper.config.offset": 0.22,
        "axis0.controller.config.pos_gain": 35.0,
        "axis0.controller.config.vel_gain": 0.25,
        "axis0.controller.config.vel_limit": 10.0,
        "axis0.controller.config.input_filter_bandwidth": 28.0,
    }
    for nid in NODE_IDS
}
# Example override:
# CONFIG_TO_WRITE[12]["axis0.controller.config.pos_gain"] = 25.0

VERIFY_PATHS = [
    # "axis0.pos_vel_mapper.config.offset",
    # "axis0.pos_estimate",
    "axis0.controller.config.pos_gain",
    "axis0.controller.config.vel_gain",
    "axis0.controller.config.vel_limit",
    "axis0.controller.config.input_filter_bandwidth",
]

STATUS_PATHS = [
    # axis / fault info
    "axis0.current_state",
    "axis0.requested_state",
    "axis0.error",
    "axis0.disarm_reason",
    "axis0.motor.error",
    "axis0.encoder.error",
    "axis0.controller.error",
    # estimates
    "axis0.pos_estimate",
    "axis0.vel_estimate",
    # supply / thermal (best effort; may not exist in your endpoints json)
    "vbus_voltage",
    "ibus",
    "axis0.fet_thermistor.temperature",
    "axis0.motor_thermistor.temperature",
    # modes
    "axis0.controller.config.control_mode",
    "axis0.controller.config.input_mode",
    # gains
    "axis0.controller.config.pos_gain",
    "axis0.controller.config.vel_gain",
    "axis0.controller.config.vel_limit",
]

DISARM_REASONS = {
    1 << 0:  "ERROR",
    1 << 1:  "MOTOR_ERROR",
    1 << 2:  "ENCODER_ERROR",
    1 << 3:  "CONTROLLER_ERROR",
    1 << 4:  "OVERVOLTAGE",
    1 << 5:  "UNDERVOLTAGE",
    1 << 6:  "OVER_TEMPERATURE",
    1 << 7:  "FET_OVER_TEMP",
    1 << 8:  "MOTOR_OVER_TEMP",
    1 << 9:  "CURRENT_LIMIT",
    1 << 10: "TORQUE_LIMIT",
    1 << 11: "VELOCITY_LIMIT",
    1 << 12: "POSITION_LIMIT",
    1 << 13: "WATCHDOG_TIMEOUT",
    1 << 14: "ESTOP",
    1 << 15: "CURRENT_LIMIT",  # ← your 32768
}


# ----------------------------
# ODrive CANSimple command IDs
# ----------------------------
# These are the standard ODrive CANSimple IDs (low 5 bits).
# They match your usage: (node_id<<5 | cmd_id).
CMD_GET_VERSION = 0x00
CMD_SET_CONTROLLER_MODE = 0x0B  # Set_Controller_Mode
# For reference:
#   RxSDO = 0x04
#   TxSDO = 0x05


# ----------------------------
# ODrive enums (common defaults)
# ----------------------------
# Control modes (ODrive firmware enums)
CONTROL_MODE_VOLTAGE = 0
CONTROL_MODE_TORQUE = 1
CONTROL_MODE_VELOCITY = 2
CONTROL_MODE_POSITION = 3

# Input modes (ODrive firmware enums)
INPUT_MODE_INACTIVE = 0
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2
INPUT_MODE_POS_FILTER = 3
INPUT_MODE_MIX_CHANNELS = 4
INPUT_MODE_TRAP_TRAJ = 5
INPUT_MODE_TORQUE_RAMP = 6
INPUT_MODE_MIRROR = 7
INPUT_MODE_TUNING = 8


# ----------------------------
# SDO constants
# ----------------------------
OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

FORMAT_LOOKUP = {
    "bool": "?",
    "uint8": "B", "int8": "b",
    "uint16": "H", "int16": "h",
    "uint32": "I", "int32": "i",
    "uint64": "Q", "int64": "q",
    "float": "f",
}


@dataclass(frozen=True)
class Endpoint:
    id: int
    typ: str


def load_endpoints(path: str) -> Dict[str, Endpoint]:
    with open(path, "r") as f:
        data = json.load(f)
    raw = data["endpoints"]
    return {k: Endpoint(id=v["id"], typ=v["type"]) for k, v in raw.items()}, data


class ODriveClient:
    """
    One client that supports:
    - CANSimple: Get_Version, Set_Controller_Mode
    - SDO: read/write endpoints, save_config, reboot
    """
    def __init__(self, bus: can.Bus, endpoints: Dict[str, Endpoint], endpoint_meta: Dict[str, Any]):
        self.bus = bus
        self.endpoints = endpoints
        self.endpoint_meta = endpoint_meta

    # ---------- Common helpers ----------
    def _flush_rx(self) -> None:
        while self.bus.recv(timeout=0) is not None:
            pass

    def _endpoint(self, path: str) -> Endpoint:
        if path not in self.endpoints:
            raise KeyError(f"Endpoint not in {ENDPOINTS_JSON}: {path}")
        return self.endpoints[path]

    # ---------- CANSimple ----------
    def check_version(self, node_id: int, timeout_s: float = 0.5, strict: bool = True) -> Dict[str, str]:
        """
        Sends CANSimple Get_Version and compares against flat_endpoints.json metadata.
        Returns dict with fw/hw versions. If strict=True, raises AssertionError on mismatch.
        """
        self._flush_rx()
        self.bus.send(can.Message(
            arbitration_id=(node_id << 5) | CMD_GET_VERSION,
            data=b"",
            is_extended_id=False
        ))

        t0 = time.time()
        while True:
            msg = self.bus.recv(timeout=0.05)
            if msg is None:
                if (time.time() - t0) > timeout_s:
                    raise TimeoutError(f"Timeout waiting for Get_Version reply from node {node_id}")
                continue
            if msg.arbitration_id == ((node_id << 5) | CMD_GET_VERSION):
                break

        # <BBBBBBBB = 8 bytes
        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack(
            "<BBBBBBBB", msg.data
        )

        fw = f"{fw_major}.{fw_minor}.{fw_revision}"
        hw = f"{hw_product_line}.{hw_version}.{hw_variant}"

        expected_fw = self.endpoint_meta.get("fw_version")
        expected_hw = self.endpoint_meta.get("hw_version")

        if strict:
            assert expected_fw == fw, f"Node {node_id} FW mismatch: expected {expected_fw}, got {fw}"
            assert expected_hw == hw, f"Node {node_id} HW mismatch: expected {expected_hw}, got {hw}"

        return {"fw_version": fw, "hw_version": hw, "fw_unreleased": str(fw_unreleased)}

    def set_controller_mode(self, node_id: int, control_mode: int, input_mode: int) -> None:
        """
        CANSimple Set_Controller_Mode: payload = <II (control_mode, input_mode)
        """
        data = struct.pack("<II", int(control_mode), int(input_mode))
        self.bus.send(can.Message(
            arbitration_id=(node_id << 5) | CMD_SET_CONTROLLER_MODE,
            data=data,
            is_extended_id=False
        ))
        print(f"Node {node_id:>2}: Set_Controller_Mode control={control_mode} input={input_mode}")

    # ---------- SDO read/write ----------
    def sdo_read(self, node_id: int, path: str, timeout_s: float = 0.25) -> Any:
        ep = self._endpoint(path)
        fmt = FORMAT_LOOKUP[ep.typ]

        self._flush_rx()
        self.bus.send(can.Message(
            arbitration_id=(node_id << 5) | 0x04,  # RxSDO
            data=struct.pack("<BHB", OPCODE_READ, ep.id, 0),
            is_extended_id=False
        ))

        t0 = time.time()
        while True:
            msg = self.bus.recv(timeout=0.02)
            if msg is None:
                if (time.time() - t0) > timeout_s:
                    raise TimeoutError(f"Timeout reading {path} from node {node_id}")
                continue
            if msg.arbitration_id == ((node_id << 5) | 0x05):  # TxSDO
                _, _, _, value = struct.unpack_from("<BHB" + fmt, msg.data)
                return value

    def sdo_write(self, node_id: int, path: str, value: Any = None) -> None:
        ep = self._endpoint(path)

        # "function" endpoints are invoked by sending only the SDO header (no value payload)
        if ep.typ == "function":
            data = struct.pack("<BHB", OPCODE_WRITE, ep.id, 0)
        else:
            fmt = FORMAT_LOOKUP[ep.typ]
            data = struct.pack("<BHB" + fmt, OPCODE_WRITE, ep.id, 0, value)

        self.bus.send(can.Message(
            arbitration_id=(node_id << 5) | 0x04,  # RxSDO
            data=data,
            is_extended_id=False
        ))

    def write_many(self, node_id: int, kv: Dict[str, Any]) -> None:
        for k, v in kv.items():
            self.sdo_write(node_id, k, v)
            print(f"Node {node_id:>2}: Set {k} = {v}")

    def read_many(self, node_id: int, paths: List[str], timeout_s: float = 0.25) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for p in paths:
            out[p] = self.sdo_read(node_id, p, timeout_s=timeout_s)
        return out

    # ---------- SDO convenience ops ----------
    def save_configuration(self, node_id: int) -> None:
        self.sdo_write(node_id, "save_configuration")  # no value
        print(f"Node {node_id:>2}: Configuration saved")

    def reboot(self, node_id: int) -> None:
        self.sdo_write(node_id, "reboot")  # no value
        print(f"Node {node_id:>2}: Reboot command sent")


# ----------------------------
# Status
# ----------------------------
def read_status(client: ODriveClient, node_id: int, timeout_s: float = 0.25) -> Dict[str, Any]:
    st = {"node_id": node_id, "state": "unknown", "fields": {}, "missing": [], "errors": []}

    for p in STATUS_PATHS:
        if p not in client.endpoints:
            st["missing"].append(p)
            continue
        try:
            st["fields"][p] = client.sdo_read(node_id, p, timeout_s=timeout_s)
        except TimeoutError as e:
            st["errors"].append(str(e))
            st["state"] = "offline"
            return st

    f = st["fields"]
    axis_err = int(f.get("axis0.error", 0) or 0)
    motor_err = int(f.get("axis0.motor.error", 0) or 0)
    enc_err = int(f.get("axis0.encoder.error", 0) or 0)
    ctrl_err = int(f.get("axis0.controller.error", 0) or 0)
    disarm = int(f.get("axis0.disarm_reason", 0) or 0)

    st["state"] = "fault" if any(x != 0 for x in [axis_err, motor_err, enc_err, ctrl_err, disarm]) else "ok"
    return st

def decode_bitmask(mask: int, table: dict):
    active = []
    for bit, name in table.items():
        if mask & bit:
            active.append(name)
    return active if active else ["NONE"]

def print_status_line(st: Dict[str, Any]) -> None:
    node_id = st["node_id"]
    state = st.get("state", "unknown")

    if state == "offline":
        print(f"Node {node_id:>2}: OFFLINE ❌  {st.get('errors')}")
        return

    f = st["fields"]
    disarm = int(f.get("axis0.disarm_reason", 0))
    disarm_txt = decode_bitmask(disarm, DISARM_REASONS)

    print(
        f"Node {node_id:>2}: {state.upper():<6} | "
        f"axis_state={f.get('axis0.current_state')} | "
        f"axis_err={f.get('axis0.error')} disarm={disarm_txt} | "
        f"pos_gain={f.get('axis0.controller.config.pos_gain')} "
        f"vel_gain={f.get('axis0.controller.config.vel_gain')} "
        f"vel_limit={f.get('axis0.controller.config.vel_limit')} | "
        f"vbus={f.get('vbus_voltage')} ibus={f.get('ibus')} | "
        f"control_mode={f.get('axis0.controller.config.control_mode')} "
        f"input_mode={f.get('axis0.controller.config.input_mode')}"
    )


# ----------------------------
# Main
# ----------------------------
def main(
    do_print_status: bool = True,
    do_check_version: bool = False,
    do_set_mode: bool = False,
    mode_control: int = CONTROL_MODE_POSITION,
    mode_input: int = INPUT_MODE_PASSTHROUGH,
    do_write_config: bool = False,
    do_verify_readback: bool = True,
    do_save_config: bool = False,
    do_reboot: bool = False,
    timeout_s: float = 0.25,
):
    endpoints, meta = load_endpoints(ENDPOINTS_JSON)
    bus = can.interface.Bus(CAN_IFACE, bustype="socketcan")
    client = ODriveClient(bus, endpoints, meta)

    for node_id in NODE_IDS:
        # 1) Status line
        if do_print_status:
            st = read_status(client, node_id, timeout_s=timeout_s)
            print_status_line(st)

        # 2) Version check
        if do_check_version:
            try:
                v = client.check_version(node_id, timeout_s=0.6, strict=True)
                print(f"Node {node_id:>2}: version OK (fw={v['fw_version']} hw={v['hw_version']})")
            except Exception as e:
                print(f"Node {node_id:>2}: version FAIL ❌  {e}")

        # 3) Set control/input mode (CANSimple)
        if do_set_mode:
            client.set_controller_mode(node_id, mode_control, mode_input)

        # 4) Write config via SDO
        if do_write_config:
            kv = CONFIG_TO_WRITE.get(node_id)
            if not kv:
                print(f"Node {node_id:>2}: no CONFIG_TO_WRITE entry, skipping write.")
            else:
                try:
                    client.write_many(node_id, kv)
                except Exception as e:
                    print(f"Node {node_id:>2}: write FAIL ❌  {e}")

        # 5) Verify readback via SDO
        if do_verify_readback:
            paths = [p for p in VERIFY_PATHS if p in endpoints]
            if paths:
                try:
                    vals = client.read_many(node_id, paths, timeout_s=timeout_s)
                    pretty = ", ".join([f"{k.split('.')[-1]}={v}" for k, v in vals.items()])
                    print(f"Node {node_id:>2}: verify: {pretty}")
                except TimeoutError as e:
                    print(f"Node {node_id:>2}: verify: OFFLINE ❌  {e}")
                except Exception as e:
                    print(f"Node {node_id:>2}: verify: ERROR ❌  {e}")

        # 6) Save config
        if do_save_config:
            try:
                client.save_configuration(node_id)
            except Exception as e:
                print(f"Node {node_id:>2}: save_configuration FAIL ❌  {e}")

        # 7) Reboot
        if do_reboot:
            try:
                client.reboot(node_id)
            except Exception as e:
                print(f"Node {node_id:>2}: reboot FAIL ❌  {e}")

        time.sleep(0.01)


if __name__ == "__main__":
    # Flip these booleans depending on what you want to do right now.
    main(
        do_print_status=True,
        do_check_version=False,     # True = verify fw/hw matches flat_endpoints.json
        
        do_set_mode=True,          # True = send CANSimple Set_Controller_Mode
        mode_control=CONTROL_MODE_POSITION,
        mode_input=INPUT_MODE_POS_FILTER,

        do_write_config=True,      # True = apply CONFIG_TO_WRITE to each node
        do_verify_readback=True,    # True = read back VERIFY_PATHS

        do_save_config=True,       # True = persist config on each node
        do_reboot=False,            # True = reboot each node

        timeout_s=0.25,
    )

# Origian gain values
# axis0.controller.config.pos_gain = 20.0
# axis0.controller.config.vel_gain = 0.1666666716337204
# axis0.controller.config.vel_limit = 10.0

# -- start load
import json
import struct
import can

# Define the node IDs and their respective configuration paths and values to set
node_ids = [11, 21, 31, 41]  # List of node IDs
configurations = {
    # 11: {'axis0.config.torque_soft_max': 1.062, 'axis0.config.torque_soft_min': -1.062},
    # 21: {'axis0.config.torque_soft_max': 1.062, 'axis0.config.torque_soft_min': -1.062},
    # 31: {'axis0.config.torque_soft_max': 1.062, 'axis0.config.torque_soft_min': -1.062},
    # 41: {'axis0.config.torque_soft_max': 1.062, 'axis0.config.torque_soft_min': -1.062}
    11: {'axis0.controller.config.input_filter_bandwidth': 20},
    21: {'axis0.controller.config.input_filter_bandwidth': 20},
    31: {'axis0.controller.config.input_filter_bandwidth': 20},
    41: {'axis0.controller.config.input_filter_bandwidth': 20}
}

# Load endpoint data
with open('flat_endpoints.json', 'r') as f:
    endpoint_data = json.load(f)
    endpoints = endpoint_data['endpoints']
# -- end load

# -- start definitions
OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

format_lookup = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}

bus = can.interface.Bus("can0", bustype="socketcan")

# -- function to check version
def check_version(node_id):
    # Flush CAN RX buffer
    while not (bus.recv(timeout=0) is None): pass

    # Send read command
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x00), # 0x00: Get_Version
        data=b'',
        is_extended_id=False
    ))

    # Await reply
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x00): # 0x00: Get_Version
            break

    _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

    current_firmware_version = f"{fw_major}.{fw_minor}.{fw_revision}"
    current_hardware_version = f"{hw_product_line}.{hw_version}.{hw_variant}"

    print(f"Node {node_id} firmware version: {current_firmware_version}")
    print(f"Node {node_id} hardware version: {current_hardware_version}")

    assert endpoint_data['fw_version'] == current_firmware_version
    assert endpoint_data['hw_version'] == current_hardware_version

# -- function to write multiple values
def write_values(node_id, config_paths):
    for path, value in config_paths.items():
        endpoint_id = endpoints[path]['id']
        endpoint_type = endpoints[path]['type']
        
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB' + format_lookup[endpoint_type], OPCODE_WRITE, endpoint_id, 0, value),
            is_extended_id=False
        ))
        print(f"Node {node_id}: Set {path} to {value}")

# -- function to read multiple values
def read_values(node_id, config_paths):
    results = {}
    for path in config_paths:
        endpoint_id = endpoints[path]['id']
        endpoint_type = endpoints[path]['type']
        
        # Flush CAN RX buffer
        while not (bus.recv(timeout=0) is None): pass

        # Send read command
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))

        # Await reply
        for msg in bus:
            if msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
                break

        # Unpack and store reply
        _, _, _, return_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)
        results[path] = return_value
        print(f"Node {node_id} received: {path} = {return_value}")
    return results

# -- function to save configuration
def save_configuration(node_id):
    path = "save_configuration"
    endpoint_id = endpoints[path]['id']

    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
        is_extended_id=False
    ))
    print(f"Node {node_id}: Configuration saved.")

# -- main loop to iterate over node IDs and configurations
for node_id in node_ids:
    # Ensure node's firmware and hardware versions match the JSON file requirements
    check_version(node_id)
    
    # Write values for each configuration path
    write_values(node_id, configurations[node_id])
    
    # Read back values to verify
    read_values(node_id, configurations[node_id])
    
    # Save configuration for the node
    save_configuration(node_id)
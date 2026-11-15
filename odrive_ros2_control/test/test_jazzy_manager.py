"""Actual Jazzy manager and Marvin Stop helper; fake CAN, no sockets or motors."""
import os
from pathlib import Path
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '226' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='isolated localhost domain226 required',
)


def test_partial_activation_normal_stop_and_retry(tmp_path):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from controller_manager_msgs.srv import LoadController, ConfigureController
    from ros2_control_utilities.hardware_state_manager import HardwareStateManager
    from RobotController.hardware_activation import HardwareState
    import yaml

    library = Path(os.environ['ODRIVE_JAZZY_FAKE_LIBRARY']).resolve(strict=True)
    prefix = tmp_path / 'prefix'
    resource = prefix / 'share/ament_index/resource_index'
    (resource / 'packages').mkdir(parents=True)
    (resource / 'packages/pi59_fake_can').touch()
    (resource / 'hardware_interface__pluginlib__plugin').mkdir()
    xml = prefix / 'share/pi59_fake_can/plugin.xml'
    xml.parent.mkdir()
    (prefix / 'lib').mkdir()
    (prefix / 'lib/libodrive_jazzy_fake_transport.so').symlink_to(library)
    (xml.parent / 'package.xml').write_text(
        '<package format="3"><name>pi59_fake_can</name><version>0.0.0</version>'
        '<description>Ticket59 fake CAN fixture</description>'
        '<maintainer email="test@example.invalid">Test</maintainer><license>MIT</license></package>')
    xml.write_text('''<library path="odrive_jazzy_fake_transport">
      <class name="pi59_fake_can/ODrive" type="odrive_ros2_control::ODriveHardwareInterface"
        base_class_type="hardware_interface::SystemInterface"/></library>''')
    (resource / 'hardware_interface__pluginlib__plugin/pi59_fake_can').write_text(
        'share/pi59_fake_can/plugin.xml\n')
    controls = tmp_path / 'controls'
    controls.mkdir()
    (controls / 'fail-enable').touch()
    params = tmp_path / 'controllers.yaml'
    ids = (11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43)
    names = [f'joint_{ident}' for ident in ids]
    params.write_text(yaml.safe_dump({
        '/pi59_driver_fixture/controller_manager': {'ros__parameters': {
            'update_rate': 200,
            'hardware_components_initial_state': {'inactive': ['TestDrives']},
            'defaults': {'allow_controller_activation_with_inactive_hardware': False},
            'joints': {'type': 'forward_command_controller_traced/ForwardCommandController'},
            'states': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        }},
        '/pi59_driver_fixture/joints': {'ros__parameters': {
            'joints': names, 'interface_name': 'position', 'export_command_receipt': True}},
    }))
    physical_joints = ''.join(
        f'<link name="tip_{ident}"/><joint name="joint_{ident}" type="revolute">'
        f'<parent link="base"/><child link="tip_{ident}"/><axis xyz="0 0 1"/>'
        '<limit lower="-3" upper="3" effort="1" velocity="1"/></joint>' for ident in ids)
    interfaces = ''.join(
        f'<joint name="joint_{ident}"><param name="node_id">{ident}</param>'
        '<command_interface name="position"/><command_interface name="command_receipt_monotonic_s"/>'
        '<state_interface name="position"/><state_interface name="velocity"/></joint>' for ident in ids)
    urdf = f'''<robot name="pi59_fixture"><link name="base"/>{physical_joints}
      <ros2_control name="TestDrives" type="system"><hardware>
        <plugin>pi59_fake_can/ODrive</plugin><param name="can">fake_only</param>
        <param name="require_command_receipt">true</param>
        <param name="can_tx_pacing">coherent_batch</param>
      </hardware>{interfaces}</ros2_control></robot>'''
    rclpy.init()
    node = rclpy.create_node('pi59_driver_test', namespace='/pi59_driver_fixture')
    manager = HardwareStateManager(node)
    publisher = node.create_publisher(String, 'robot_description',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log = (tmp_path / 'controller-manager.log').open('w')
    process = None

    def call(service, leaf, request):
        client = node.create_client(service, 'controller_manager/' + leaf)
        try:
            assert client.wait_for_service(timeout_sec=10), leaf
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done() and future.result().ok, leaf
        finally:
            node.destroy_client(client)

    def frames():
        return [tuple(map(int, line.split())) for line in (controls / 'frames.log').read_text().splitlines()]

    def idle_count():
        return sum(command == 7 and state == 1 for _, command, state in frames())

    try:
        env = dict(os.environ, ODRIVE_JAZZY_FAKE_CONTROL_DIR=str(controls),
            AMENT_PREFIX_PATH=str(prefix) + ':' + os.environ.get('AMENT_PREFIX_PATH', ''))
        process = subprocess.Popen([
            '/opt/ros/jazzy/lib/controller_manager/ros2_control_node', '--ros-args',
            '-r', '__ns:=/pi59_driver_fixture', '--params-file', str(params),
        ], stdout=log, stderr=subprocess.STDOUT, env=env)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            assert process.poll() is None, 'fake manager exited'
            publisher.publish(String(data=urdf))
            rclpy.spin_once(node, timeout_sec=.05)
            if manager.get_hardware_state('TestDrives') == 'inactive':
                break
        assert manager.get_hardware_state('TestDrives') == 'inactive'
        for name in ('states', 'joints'):
            call(LoadController, 'load_controller', LoadController.Request(name=name))
            call(ConfigureController, 'configure_controller', ConfigureController.Request(name=name))
        assert manager._switch_controllers(activate=('states',))
        before = idle_count()
        result = manager.activate_motion('TestDrives', ('joints',), ('states',), lambda: True)
        assert not result.ready and result.hardware is HardwareState.UNKNOWN, result
        assert manager.get_hardware_state('TestDrives') == 'unconfigured'
        time.sleep(.1)
        assert idle_count() == before, 'partial failure must not send automatic IDLE'
        recent = frames()
        assert sum(command == 12 for _, command, _ in recent) > 24, 'preclaim support must continue'
        # Existing supervisor admission must refuse a retry, with no configure/enable.
        before_retry = len(frames())
        result = manager.activate_motion('TestDrives', ('joints',), ('states',), lambda: True)
        assert not result.ready
        assert not any(command in (7, 11) for _, command, _ in frames()[before_retry:])
        (controls / 'silent-idle').touch()
        assert not manager.set_hardware_to_idle('TestDrives', ('joints',))
        assert manager.get_hardware_state('TestDrives') == 'unconfigured'
        assert idle_count() > before, 'normal Stop must reach verified-IDLE implementation'
        (controls / 'silent-idle').unlink()
        assert manager.set_hardware_to_idle('TestDrives', ('joints',))
        assert manager.get_hardware_state('TestDrives') == 'inactive'
        (controls / 'fail-enable').unlink()
        time.sleep(.05)
        result = manager.activate_motion('TestDrives', ('joints',), ('states',), lambda: True)
        assert result.ready, result.reason
        assert manager.controller_states() == {'joints': 'active', 'states': 'active'}
        assert manager.set_hardware_to_idle('TestDrives', ('joints',))
        assert manager.get_hardware_state('TestDrives') == 'inactive'
    finally:
        manager.close()
        node.destroy_node()
        rclpy.try_shutdown()
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        log.close()
        assert process is None or process.poll() is not None

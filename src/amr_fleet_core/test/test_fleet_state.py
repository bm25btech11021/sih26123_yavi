"""Unit tests for FleetState and RobotInfo abstractions."""

import os
import tempfile

from amr_fleet_core import FleetState, RobotInfo


def test_robot_info_creation_and_dict():
    """Verify RobotInfo instantiation and dictionary serialization."""
    info = RobotInfo(
        robot_id='amr_0',
        namespace='/amr_0',
        pose_topic='/amr_0/odom',
        cmd_vel_topic='/amr_0/cmd_vel',
        scan_topic='/amr_0/scan',
        status='ACTIVE',
        initial_pose=(2.0, 2.0, 0.15, 0.0),
    )
    d = info.to_dict()
    assert d['robot_id'] == 'amr_0'
    assert d['namespace'] == '/amr_0'
    assert d['pose_topic'] == '/amr_0/odom'
    assert d['cmd_vel_topic'] == '/amr_0/cmd_vel'
    assert d['scan_topic'] == '/amr_0/scan'
    assert d['status'] == 'ACTIVE'
    assert d['initial_pose'] == [2.0, 2.0, 0.15, 0.0]


def test_fleet_state_manual_registration():
    """Verify adding, querying, and listing robots in FleetState."""
    state = FleetState()
    assert state.count() == 0
    assert not state.contains('amr_0')

    r0 = RobotInfo('amr_0', '/amr_0', '/amr_0/odom', '/amr_0/cmd_vel', '/amr_0/scan')
    r1 = RobotInfo('amr_1', '/amr_1', '/amr_1/odom', '/amr_1/cmd_vel', '/amr_1/scan')
    state.add_robot(r0)
    state.add_robot(r1)

    assert state.count() == 2
    assert state.contains('amr_0')
    assert state.contains('amr_1')
    assert not state.contains('amr_2')
    assert state.get_robot('amr_0') == r0
    assert state.active_robot_ids() == ['amr_0', 'amr_1']


def test_fleet_state_from_yaml():
    """Verify constructing FleetState from a YAML configuration file."""
    yaml_content = """
fleet:
  world: "warehouse_small"
  robots:
    - id: "amr_0"
      x: 1.0
      y: 2.0
      z: 0.15
      yaw: 0.0
    - id: "amr_1"
      x: 3.0
      y: 4.0
      z: 0.15
      yaw: 1.57
    - id: "amr_2"
      x: 5.0
      y: 6.0
      z: 0.15
      yaw: 3.14
"""
    with tempfile.NamedTemporaryFile('w', delete=False, suffix='.yaml') as f:
        f.write(yaml_content)
        tmp_path = f.name

    try:
        state_all = FleetState.from_yaml(tmp_path)
        assert state_all.count() == 3
        assert state_all.active_robot_ids() == ['amr_0', 'amr_1', 'amr_2']
        r1 = state_all.get_robot('amr_1')
        assert r1 is not None
        assert r1.initial_pose == (3.0, 4.0, 0.15, 1.57)

        # Test with max_count limit
        state_two = FleetState.from_yaml(tmp_path, max_count=2)
        assert state_two.count() == 2
        assert state_two.active_robot_ids() == ['amr_0', 'amr_1']
    finally:
        os.remove(tmp_path)


def test_fleet_state_from_topics():
    """Verify dynamic discovery of robots from active ROS 2 topic names."""
    topics = [
        '/clock',
        '/tf',
        '/tf_static',
        '/amr_0/cmd_vel',
        '/amr_0/odom',
        '/amr_0/scan',
        '/amr_1/cmd_vel',
        '/amr_1/odom',
        '/amr_1/scan',
        '/parameter_events',
        '/rosout',
    ]
    state = FleetState.from_topics(topics)
    assert state.count() == 2
    assert state.contains('amr_0')
    assert state.contains('amr_1')
    assert state.get_robot('amr_0').pose_topic == '/amr_0/odom'
    assert state.get_robot('amr_1').cmd_vel_topic == '/amr_1/cmd_vel'


"""Unit tests for M9-V2 Pillar C: Deterministic Local Safety Backstop."""

import math

from amr_fleet_core.rh_node import RollingHorizonPlannerNode
import rclpy


def test_peer_proximity_hazard_directly_ahead():
    """Verify emergency peer hazard triggers when peer is in forward corridor <= 0.700m."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        # amr_4 pose in Pilot 2: at (14.75, 15.28), facing south (-pi/2)
        node.planner.current_position = (14.75, 15.28)
        node.current_yaw = -math.pi / 2.0

        # Lead vehicle amr_3 at (14.75, 14.94) (distance = 0.34m)
        node.peer_states = {
            'amr_3': {
                'pose': (14.75, 14.94),
                'current_cell': (29, 29),
                'target_cell': (29, 29),
            }
        }

        assert node._check_peer_proximity_hazard() is True, \
            'Hazard must trigger for peer 0.34m directly ahead'
    finally:
        node.destroy_node()


def test_peer_proximity_hazard_peer_behind():
    """Verify hazard does NOT trigger when peer is behind the robot."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        node.planner.current_position = (14.75, 15.28)
        node.current_yaw = -math.pi / 2.0

        # Peer is north of the robot at y=16.0 (behind robot moving south)
        node.peer_states = {
            'amr_2': {
                'pose': (14.75, 16.00),
                'current_cell': (29, 32),
                'target_cell': (29, 32),
            }
        }

        assert node._check_peer_proximity_hazard() is False, \
            'Hazard must NOT trigger for peer behind robot'
    finally:
        node.destroy_node()


def test_peer_proximity_hazard_lateral_aisle():
    """Verify hazard does NOT trigger for peer in parallel aisle (> 0.40m lateral)."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        node.planner.current_position = (14.75, 15.28)
        node.current_yaw = -math.pi / 2.0

        # Peer is in next aisle at x=16.75 (lateral distance = 2.0m)
        node.peer_states = {
            'amr_5': {
                'pose': (16.75, 14.94),
                'current_cell': (33, 29),
                'target_cell': (33, 29),
            }
        }

        assert node._check_peer_proximity_hazard() is False, \
            'Hazard must NOT trigger for peer in adjacent parallel aisle'
    finally:
        node.destroy_node()


def test_peer_proximity_hazard_safe_headway():
    """Verify hazard does NOT trigger when peer is > 0.700m ahead."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        node.planner.current_position = (14.75, 15.28)
        node.current_yaw = -math.pi / 2.0

        # Peer is 1.0m ahead at y=14.28
        node.peer_states = {
            'amr_3': {
                'pose': (14.75, 14.28),
                'current_cell': (29, 28),
                'target_cell': (29, 28),
            }
        }

        assert node._check_peer_proximity_hazard() is False, \
            'Hazard must NOT trigger when peer is beyond 0.700m threshold'
    finally:
        node.destroy_node()


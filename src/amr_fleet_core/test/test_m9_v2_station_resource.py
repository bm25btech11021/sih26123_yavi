"""Unit tests for M9-V2 Pillar D: Central Delivery Bay as Shared Resource."""

import math

from amr_fleet_core.coordination_models import CoordinationState
from amr_fleet_core.rh_node import RollingHorizonPlannerNode
import rclpy


def test_station_staging_buffer_holds_when_bay_occupied():
    """Verify follower holds at staging cell (29, 33) while Bay 1 is occupied."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        # amr_3 is currently in Bay 1 (29, 29)
        node.peer_states = {
            'amr_3': {
                'pose': (14.75, 14.75),
                'current_cell': (29, 29),
                'target_cell': (29, 29),
                'priority': 500.0,
            }
        }

        # amr_4 is at holding cell (29, 33) trying to advance to (29, 32)
        curr_cell = (29, 33)
        desired_next_cell = (29, 32)
        goal_cell = (29, 29)

        safe_cell, state, wait_for = node._evaluate_local_coordination(
            curr_cell, desired_next_cell, goal_cell,
        )

        assert safe_cell == (29, 33), 'Robot must remain at holding cell (29, 33)'
        assert state == CoordinationState.YIELDING.value, 'State must be YIELDING'
        assert wait_for == 'amr_3', 'Waiting for lead robot amr_3'

        # Verify safe separation at holding cell
        p_hold = node.grid.to_world((29, 33))
        p_bay = node.grid.to_world((29, 29))
        dist = math.hypot(p_hold[0] - p_bay[0], p_hold[1] - p_bay[1])
        assert dist == 2.00, f'Expected 2.0m discrete separation, got {dist}'
        bumper_clearance = dist - 0.65
        assert bumper_clearance == 1.35, f'Expected 1.35m clearance, got {bumper_clearance}'
    finally:
        node.destroy_node()


def test_station_staging_releases_when_bay_cleared():
    """Verify that follower is released to proceed once lead AMR clears Bay 1."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        # amr_3 has vacated Bay 1 (now at (25, 29))
        node.peer_states = {
            'amr_3': {
                'pose': (12.75, 14.75),
                'current_cell': (25, 29),
                'target_cell': (25, 29),
                'priority': 500.0,
            }
        }

        curr_cell = (29, 33)
        desired_next_cell = (29, 32)
        goal_cell = (29, 29)

        safe_cell, state, wait_for = node._evaluate_local_coordination(
            curr_cell, desired_next_cell, goal_cell,
        )

        assert safe_cell == (29, 32), 'Robot must be allowed to advance'
        assert state == CoordinationState.CLEAR.value, 'State must be CLEAR'
        assert wait_for == '', 'Not waiting for any robot'
    finally:
        node.destroy_node()


def test_station_staging_bypassed_for_other_destinations():
    """Verify staging does not block robots targeting different goals."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        node.peer_states = {
            'amr_3': {
                'pose': (14.75, 14.75),
                'current_cell': (29, 29),
                'target_cell': (29, 29),
                'priority': 500.0,
            }
        }

        # Robot targeting another destination, e.g. (20, 20)
        curr_cell = (29, 33)
        desired_next_cell = (29, 32)
        goal_cell = (20, 20)

        safe_cell, state, _ = node._evaluate_local_coordination(
            curr_cell, desired_next_cell, goal_cell,
        )

        assert safe_cell == (29, 32)
        assert state == CoordinationState.CLEAR.value
    finally:
        node.destroy_node()


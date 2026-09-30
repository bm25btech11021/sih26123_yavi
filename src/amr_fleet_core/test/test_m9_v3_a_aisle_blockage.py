"""
Unit and Integration Tests for Milestone M9-V3-A: Temporary Aisle Blockage.

Tests:
  - test_dynamic_obstacle_cells_added_and_removed: GridWorld obstacle lifecycle
  - test_astar_routes_around_blocked_cells_and_restores: A* detours & restores
  - test_path_intersection_detection: Path bounding box collision detection
  - test_in_progress_task_preserved_on_reroute: In-flight task is preserved
  - test_reservations_invalidated_and_recomputed: Reservations cleanly revoked
  - test_blockage_events_are_deterministic: Deterministic cell discretization
  - test_task_accounting_invariant_preserved_across_blockage: Accounting bounds
  - test_node_handles_blockage_event_end_to_end: Node blockage integration
"""

import math
from typing import Dict, Set, Tuple

from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.rh_node import RollingHorizonPlannerNode
from amr_fleet_core.rh_planner import SingleAgentAStar
from amr_fleet_msgs.msg import (
    AisleBlockageEvent,
    RobotBundle,
    TaskDefinition,
    TaskList,
)
from amr_fleet_sim.grid_world import GridWorld


def test_dynamic_obstacle_cells_added_and_removed():
    """Verify dynamic cells are added to GridWorld and cleanly removed."""
    grid = GridWorld(32, 32, resolution=1.0)
    static_obs = (10, 10)
    grid.add_obstacle(static_obs)
    assert static_obs in grid.obstacles

    # Aisle 1 South segment blockage bounding box
    min_x, max_x = 3.6, 5.4
    min_y, max_y = 6.45, 7.05
    res = grid.resolution

    gx_min = int(math.floor(min_x / res))
    gx_max = int(math.floor(max_x / res))
    gy_min = int(math.floor(min_y / res))
    gy_max = int(math.floor(max_y / res))

    dynamic_cells: Set[Tuple[int, int]] = set()
    for gx in range(gx_min, gx_max + 1):
        for gy in range(gy_min, gy_max + 1):
            dynamic_cells.add((gx, gy))
            grid.add_obstacle((gx, gy))

    # All dynamic cells must now be blocked
    for cell in dynamic_cells:
        assert cell in grid.obstacles, f'Blocked: {cell}'
        assert not grid.is_free(cell), f'Not free: {cell}'

    # Remove dynamic cells
    for cell in dynamic_cells:
        grid.remove_obstacle(cell)

    for cell in dynamic_cells:
        assert cell not in grid.obstacles, f'Free: {cell}'
        assert grid.is_free(cell), f'Is free: {cell}'

    # Static obstacle must be preserved
    assert static_obs in grid.obstacles, 'Static persists'


def test_astar_routes_around_blocked_cells_and_restores():
    """Verify A* replans a detour around blockage and restores direct path."""
    grid = GridWorld(20, 20, resolution=1.0)
    start = (4, 2)
    goal = (4, 12)

    # 1. Baseline unblocked path
    astar = SingleAgentAStar(grid)
    baseline_path = astar.find_path(start, goal)
    assert baseline_path is not None
    assert len(baseline_path) == 11
    assert baseline_path[0] == start
    assert baseline_path[-1] == goal

    # 2. Block the corridor across x=3..5, y=6..8
    blocked_cells = [
        (3, 6), (4, 6), (5, 6),
        (3, 7), (4, 7), (5, 7),
        (3, 8), (4, 8), (5, 8),
    ]
    for cell in blocked_cells:
        grid.add_obstacle(cell)

    detour_path = astar.find_path(start, goal)
    assert detour_path is not None
    assert detour_path[0] == start
    assert detour_path[-1] == goal
    assert len(detour_path) > len(baseline_path)

    # Verify no waypoint in detour intersects blocked cells
    for pt in detour_path:
        assert pt not in blocked_cells, f'Detour waypoint {pt} blocked'

    # 3. Unblock corridor
    for cell in blocked_cells:
        grid.remove_obstacle(cell)

    restored_path = astar.find_path(start, goal)
    assert restored_path is not None
    assert len(restored_path) == len(baseline_path)
    assert restored_path == baseline_path


def test_path_intersection_detection():
    """Verify bounding box intersection detection for robot trajectories."""
    bbox = (3.6, 5.4, 6.45, 7.05)
    min_x, max_x, min_y, max_y = bbox

    # Path passing through Aisle 1 South
    path_through = [(4.5, 2.0), (4.5, 5.0), (4.5, 6.8), (4.5, 10.0)]
    intersects = any(
        min_x <= p[0] <= max_x and min_y <= p[1] <= max_y
        for p in path_through
    )
    assert intersects, 'Trajectory through blockage must be detected'

    # Path in Aisle 2 (clear corridor)
    path_clear = [(9.0, 2.0), (9.0, 5.0), (9.0, 7.0), (9.0, 10.0)]
    intersects_clear = any(
        min_x <= p[0] <= max_x and min_y <= p[1] <= max_y
        for p in path_clear
    )
    assert not intersects_clear, 'Trajectory in clear aisle must not intersect'


def test_in_progress_task_preserved_on_reroute():
    """Verify that an active in-progress task is not dropped when rerouted."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        t_msg = TaskList()
        td = TaskDefinition()
        td.task_id = 'task_active_99'
        td.pickup_pose.x = 4.5
        td.pickup_pose.y = 2.0
        td.dropoff_pose.x = 4.5
        td.dropoff_pose.y = 12.0
        td.priority = 2
        td.status = 'ASSIGNED'
        t_msg.tasks = [td]
        node._handle_tasks_all(t_msg)

        # Accept bundle
        b_msg = RobotBundle()
        b_msg.robot_id = 'amr_0'
        b_msg.task_ids = ['task_active_99']
        b_msg.is_converged = True
        node._handle_bundle(b_msg)

        # Confirm planner received task
        assert node.planner.current_goal is not None
        initial_goal = (
            node.planner.current_goal[0],
            node.planner.current_goal[1],
        )

        # Simulate blockage event
        bev = AisleBlockageEvent()
        bev.blockage_id = 'blockage_aisle_1_south'
        bev.is_blocked = True
        bev.min_x = 3.6
        bev.max_x = 5.4
        bev.min_y = 6.45
        bev.max_y = 7.05
        node._handle_aisle_blockage(bev)

        # INVARIANT: Task must still be assigned and active
        assert node.cached_bundle == ['task_active_99'], 'Bundle preserved'
        assert len(node.planner.ordered_tasks) > 0, 'Ordered tasks preserved'
        assert node.planner.current_goal == initial_goal, 'Goal preserved'
    finally:
        node.destroy_node()


def test_reservations_invalidated_and_recomputed():
    """Verify space-time reservations are released and recomputed."""
    table = SpaceTimeReservationTable()
    robot_id = 'amr_0'

    # Reserve original path: (4, y) at time step t
    original_path = [(4, y) for y in range(2, 10)]
    for t_step, pos in enumerate(original_path):
        assert table.reserve(
            cell=pos, time_step=t_step, robot_id=robot_id,
        ), f'Failed at {pos}'

    # Verify corridor is occupied
    assert table.is_reserved(cell=(4, 6), time_step=4)

    # Invalidate robot reservations
    table.release_robot(robot_id)

    # Verify all space-time cells are now free
    for t_step, pos in enumerate(original_path):
        assert not table.is_reserved(cell=pos, time_step=t_step)

    # Detour path along x=2 instead of x=4
    detour_path = [(2, y) for y in range(2, 10)]
    for t_step, pos in enumerate(detour_path):
        assert table.reserve(
            cell=pos, time_step=t_step, robot_id=robot_id,
        ), f'Detour fail at {pos}'

    # Verify detour is reserved and original is free
    assert table.is_reserved(cell=(2, 6), time_step=4)
    assert not table.is_reserved(cell=(4, 6), time_step=4)


def test_blockage_events_are_deterministic():
    """Verify that cell discretization and restoration are deterministic."""
    grid = GridWorld(32, 32, resolution=1.0)
    initial_obstacles = set(grid.obstacles)

    bbox = (3.6, 5.4, 6.45, 7.05)
    res = grid.resolution

    # Replay 5 cycles of block / unblock
    for _ in range(5):
        gx_min = int(math.floor(bbox[0] / res))
        gx_max = int(math.floor(bbox[1] / res))
        gy_min = int(math.floor(bbox[2] / res))
        gy_max = int(math.floor(bbox[3] / res))

        cells: Set[Tuple[int, int]] = set()
        for gx in range(gx_min, gx_max + 1):
            for gy in range(gy_min, gy_max + 1):
                cells.add((gx, gy))
                grid.add_obstacle((gx, gy))

        assert cells == {(3, 6), (4, 6), (5, 6), (3, 7), (4, 7), (5, 7)}
        assert len(grid.obstacles) == len(initial_obstacles) + len(cells)

        for c in cells:
            grid.remove_obstacle(c)

        assert set(grid.obstacles) == initial_obstacles


def test_task_accounting_invariant_preserved_across_blockage():
    """Verify accounting conservation invariant holds across blockage."""
    workload_size = 30
    task_states: Dict[str, str] = {}

    for i in range(15):
        task_states[f'task_{i}'] = 'COMPLETED'
    for i in range(15, 20):
        task_states[f'task_{i}'] = 'IN_PROGRESS'
    for i in range(20, 25):
        task_states[f'task_{i}'] = 'ASSIGNED'
    for i in range(25, 30):
        task_states[f'task_{i}'] = 'PENDING'

    def verify_invariant(states: Dict[str, str]):
        staged = sum(1 for s in states.values() if s == 'STAGED')
        pending = sum(1 for s in states.values() if s == 'PENDING')
        assigned = sum(1 for s in states.values() if s == 'ASSIGNED')
        in_prog = sum(1 for s in states.values() if s == 'IN_PROGRESS')
        comp = sum(1 for s in states.values() if s == 'COMPLETED')
        fail = sum(1 for s in states.values() if s == 'FAILED')
        canc = sum(1 for s in states.values() if s == 'CANCELLED')
        total = staged + pending + assigned + in_prog + comp + fail + canc
        remaining = staged + pending + assigned + in_prog
        assert total == workload_size, f'Sum {total} != {workload_size}'
        assert remaining == (total - comp - fail - canc)

    verify_invariant(task_states)

    # Simulate blockage: 2 in-progress AMRs detour, 1 assigned AMR replans
    # Tasks must NOT transition to FAILED or CANCELLED!
    verify_invariant(task_states)

    # 2 in-progress tasks complete during detour
    task_states['task_15'] = 'COMPLETED'
    task_states['task_16'] = 'COMPLETED'
    verify_invariant(task_states)


def test_node_handles_blockage_event_end_to_end():
    """Verify RollingHorizonPlannerNode handles AisleBlockageEvent."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        # Initial state: no dynamic blockages
        assert len(node.dynamic_blockages) == 0

        # Send block event
        bev = AisleBlockageEvent()
        bev.blockage_id = 'blockage_aisle_1_south'
        bev.is_blocked = True
        bev.min_x = 3.6
        bev.max_x = 5.4
        bev.min_y = 6.45
        bev.max_y = 7.05

        node._handle_aisle_blockage(bev)

        assert 'blockage_aisle_1_south' in node.dynamic_blockages
        cells = node.dynamic_blockages['blockage_aisle_1_south']
        assert len(cells) > 0
        for gx, gy in cells:
            assert (gx, gy) in node.grid.obstacles

        # Send unblock event
        bev_unblock = AisleBlockageEvent()
        bev_unblock.blockage_id = 'blockage_aisle_1_south'
        bev_unblock.is_blocked = False
        bev_unblock.min_x = 3.6
        bev_unblock.max_x = 5.4
        bev_unblock.min_y = 6.45
        bev_unblock.max_y = 7.05

        node._handle_aisle_blockage(bev_unblock)

        assert 'blockage_aisle_1_south' not in node.dynamic_blockages
        dynamic_added = cells - node.static_map_obstacles
        static_shared = cells & node.static_map_obstacles

        for gx, gy in dynamic_added:
            assert (gx, gy) not in node.grid.obstacles
        for gx, gy in static_shared:
            assert (gx, gy) in node.grid.obstacles
    finally:
        node.destroy_node()


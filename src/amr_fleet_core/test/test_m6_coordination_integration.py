"""Integration tests connecting M5 RollingHorizonPlanner with M6 MultiAgentCoordinator."""

from amr_fleet_core.conflict_detector import ConflictDetector
from amr_fleet_core.multi_agent_coordinator import MultiAgentCoordinator
from amr_fleet_core.rh_planner import RHConfig, RollingHorizonPlanner
from amr_fleet_sim.grid_world import GridWorld


def test_m5_to_m6_vertex_conflict_resolution() -> None:
    """
    Test that two M5 planners converging on the same cell are coordinated by M6.

    Higher priority robot reaches cell first; lower priority robot yields.
    """
    grid = GridWorld.create_warehouse_grid(resolution=0.5)
    coord = MultiAgentCoordinator(grid=grid)

    # amr_0 at (2.0, 2.0) with High Priority task (priority=3)
    p0 = RollingHorizonPlanner('amr_0', config=RHConfig(grid_resolution=0.5))
    p0.update_position((2.0, 2.0))
    p0.update_assigned_bundle(
        ['t_high'],
        {'t_high': {'pickup': (4.0, 2.0), 'dropoff': (6.0, 2.0), 'priority': 3}},
    )

    # amr_1 at (4.0, 4.0) with Low Priority task (priority=1) heading to same cell (4.0, 2.0)
    p1 = RollingHorizonPlanner('amr_1', config=RHConfig(grid_resolution=0.5))
    p1.update_position((4.0, 4.0))
    p1.update_assigned_bundle(
        ['t_low'],
        {'t_low': {'pickup': (4.0, 2.0), 'dropoff': (4.0, 6.0), 'priority': 1}},
    )

    fleet = {'amr_0': p0, 'amr_1': p1}

    # Execute coordination
    coord_paths = coord.coordinate_fleet(fleet, current_time_step=0, window_size=4)

    # Verify that coordinated execution paths have ZERO conflicts
    conflicts = ConflictDetector.check_fleet_trajectories(coord_paths, start_time_step=0)
    assert len(conflicts) == 0

    # Verify metrics
    metrics = coord.get_metrics_summary()
    assert metrics['total_coordination_cycles'] == 1
    assert metrics['average_coordination_latency_ms'] < 10.0


def test_m5_to_m6_edge_swap_prevention() -> None:
    """Test that head-on confrontation in a narrow corridor is prevented from edge-swapping."""
    # Create narrow 1-tile corridor between obstacles
    # Corridor along y=3 from x=2 to x=5
    obstacles = [(x, 2) for x in range(2, 6)] + [(x, 4) for x in range(2, 6)]
    grid = GridWorld(10, 10, obstacles=obstacles, resolution=1.0)
    coord = MultiAgentCoordinator(grid=grid)

    # amr_0 at (2, 3) moving to (5, 3), priority=3
    p0 = RollingHorizonPlanner('amr_0', config=RHConfig(grid_resolution=1.0))
    p0.update_position((2.0, 3.0))
    p0.update_assigned_bundle(
        ['t_east'],
        {'t_east': {'pickup': (5.0, 3.0), 'dropoff': (6.0, 3.0), 'priority': 3}},
    )

    # amr_1 at (5, 3) moving to (2, 3), priority=1
    p1 = RollingHorizonPlanner('amr_1', config=RHConfig(grid_resolution=1.0))
    p1.update_position((5.0, 3.0))
    p1.update_assigned_bundle(
        ['t_west'],
        {'t_west': {'pickup': (2.0, 3.0), 'dropoff': (1.0, 3.0), 'priority': 1}},
    )

    fleet = {'amr_0': p0, 'amr_1': p1}

    # Coordinate 4-step window
    coord_paths = coord.coordinate_fleet(fleet, current_time_step=0, window_size=4)

    # Edge swap MUST be prevented
    conflicts = ConflictDetector.check_fleet_trajectories(coord_paths, start_time_step=0)
    for c in conflicts:
        assert c.conflict_type != 'EDGE_SWAP'


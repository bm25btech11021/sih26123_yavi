"""Unit tests for M9-V2 Pillar A: Waypoint / Task-Goal Consistency."""

import math
import os

from amr_fleet_core.rh_planner import RHConfig, RollingHorizonPlanner
from amr_fleet_sim.grid_world import GridWorld
import yaml


def _load_v2_grid():
    cur = os.path.dirname(os.path.abspath(__file__))
    repo_root = cur
    for _ in range(5):
        if os.path.isdir(os.path.join(repo_root, 'config', 'maps')):
            break
        repo_root = os.path.dirname(repo_root)

    map_path = os.path.join(repo_root, 'config', 'maps', 'warehouse_m9_v2.yaml')
    with open(map_path, 'r', encoding='utf-8') as f:
        map_data = yaml.safe_load(f)
    grid = GridWorld.from_yaml(map_path)
    return grid, map_data


def test_all_16_stations_target_continuous_coordinates():
    """All 16 stations must have world_path[-1] equal to continuous coordinates."""
    grid, map_data = _load_v2_grid()
    pickups = map_data.get('stations', {}).get('pickups', [])
    dropoffs = map_data.get('stations', {}).get('dropoffs', [])
    all_stations = pickups + dropoffs
    assert len(all_stations) == 16, 'Expected 16 total stations (12 pickups, 4 dropoffs)'

    config = RHConfig(
        horizon_steps=10,
        execution_window=4,
        grid_resolution=0.5,
        goal_tolerance_m=0.5,
    )
    planner = RollingHorizonPlanner(robot_id='amr_0', config=config, grid=grid)

    for st in all_stations:
        st_id = st['id']
        coords = tuple(st['coords'])

        planner.update_position((16.0, 16.0))
        planner.current_goal = coords
        planner.active_phase = 'TRANSIT_TO_DROPOFF'

        resp = planner.replan()
        assert resp.success, f'Failed to plan to station {st_id} at {coords}'
        assert len(planner.full_path) > 0

        term_wp = planner.full_path[-1]
        assert abs(term_wp[0] - coords[0]) < 1e-3, f'Station {st_id} X mismatch'
        assert abs(term_wp[1] - coords[1]) < 1e-3, f'Station {st_id} Y mismatch'


def test_pilot_2_stall_condition_resolved():
    """Verify amr_3 stall condition at (14.75, 14.94) drives to (14.5, 14.5)."""
    grid, _ = _load_v2_grid()
    config = RHConfig(
        horizon_steps=10,
        execution_window=4,
        grid_resolution=0.5,
        goal_tolerance_m=0.5,
    )
    planner = RollingHorizonPlanner(robot_id='amr_3', config=config, grid=grid)
    lead_pos = (14.75, 14.94)
    bay_1 = (14.5, 14.5)

    dist_to_goal = math.hypot(lead_pos[0] - bay_1[0], lead_pos[1] - bay_1[1])
    assert dist_to_goal > 0.500, 'Should exceed 0.50m goal tolerance initially'

    planner.update_position(lead_pos)
    planner.current_goal = bay_1
    planner.active_phase = 'TRANSIT_TO_DROPOFF'

    assert not planner.check_subgoal_arrival(), 'Should not be arrived at (14.75, 14.94)'

    resp = planner.replan()
    assert resp.success
    assert len(planner.execution_path) == 2
    assert planner.execution_path[0] == lead_pos
    assert planner.execution_path[1] == bay_1

    target_wp = planner.get_current_target_waypoint()
    assert target_wp == bay_1, 'Target waypoint must be the exact continuous goal'

    arrived_pos = (14.55, 14.55)
    planner.update_position(arrived_pos)
    assert planner.check_subgoal_arrival(), 'Should satisfy arrival within tolerance'


def test_replan_trigger_on_waypoint_exhaustion():
    """When execution window is exhausted, check_replan_triggers must evaluate True."""
    grid, _ = _load_v2_grid()
    config = RHConfig(
        horizon_steps=10,
        execution_window=4,
        grid_resolution=0.5,
        goal_tolerance_m=0.5,
    )
    planner = RollingHorizonPlanner(robot_id='amr_1', config=config, grid=grid)
    planner.update_position((14.0, 14.0))
    planner.current_goal = (14.5, 14.5)
    planner.active_phase = 'TRANSIT_TO_DROPOFF'
    planner.replan()

    assert len(planner.execution_path) > 1
    # Advance until exhausted
    for _ in range(len(planner.execution_path) + 1):
        planner.advance_waypoint()

    assert planner.check_replan_triggers()


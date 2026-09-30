"""Unit tests for PIBT Prioritized Local Planning (M6)."""

from typing import List

from amr_fleet_core.coordination_models import Position
from amr_fleet_core.pibt_planner import PIBTAgentState, PIBTLocalPlanner
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_sim.grid_world import GridWorld


def test_candidate_generation() -> None:
    """Test that candidate neighbor generation prioritizes goal and A* guidance."""
    grid = GridWorld(10, 10)
    table = SpaceTimeReservationTable()
    planner = PIBTLocalPlanner(grid, table)

    # Agent at (2, 2), goal at (5, 2), preferred waypoint (3, 2)
    agent = PIBTAgentState(
        robot_id='amr_0',
        current_pos=(2, 2),
        goal_pos=(5, 2),
        preferred_path=[(3, 2), (4, 2), (5, 2)],
    )
    candidates = planner.generate_candidate_moves(agent, curr_pos=(2, 2))

    # Preferred next waypoint along path should be first candidate
    assert candidates[0] == (3, 2)
    # Wait action should be included
    assert (2, 2) in candidates


def test_deterministic_priorities() -> None:
    """Test that priority computation is strictly deterministic and ordered."""
    grid = GridWorld(10, 10)
    # High task priority (3) vs Low task priority (1)
    agent_high = PIBTAgentState('amr_0', current_pos=(1, 1), goal_pos=(8, 8), task_priority=3)
    agent_low = PIBTAgentState('amr_1', current_pos=(1, 1), goal_pos=(8, 8), task_priority=1)
    assert agent_high.compute_priority(grid) > agent_low.compute_priority(grid)

    # Identical task priority and goal distance: tie-broken by robot_id
    a0 = PIBTAgentState('amr_0', current_pos=(1, 1), goal_pos=(5, 5), task_priority=2)
    a1 = PIBTAgentState('amr_1', current_pos=(1, 1), goal_pos=(5, 5), task_priority=2)
    assert a0.compute_priority(grid) > a1.compute_priority(grid)


def test_priority_inheritance_push() -> None:
    """Test priority inheritance push where higher priority robot clears occupant."""
    grid = GridWorld(10, 10)
    table = SpaceTimeReservationTable()
    planner = PIBTLocalPlanner(grid, table)

    # Robot A at (2, 2), goal at (2, 5), higher priority (3)
    agent_a = PIBTAgentState(
        'amr_0', current_pos=(2, 2), goal_pos=(2, 5), task_priority=3,
        preferred_path=[(2, 3), (2, 4), (2, 5)],
    )
    # Robot B at (2, 3), goal at (8, 8), lower priority (1)
    agent_b = PIBTAgentState(
        'amr_1', current_pos=(2, 3), goal_pos=(8, 8), task_priority=1,
    )

    agents = {'amr_0': agent_a, 'amr_1': agent_b}
    moves, conflicts = planner.plan_step(agents, time_step=0)

    # amr_0 successfully claimed (2, 3)
    assert moves['amr_0'] == (2, 3)
    # amr_1 vacated (2, 3) and moved to an alternative neighbor (e.g. (3, 3) or (2, 4))
    assert moves['amr_1'] != (2, 3)
    assert moves['amr_1'] in grid.get_neighbors((2, 3), allow_wait=False)


def test_pibt_backtracking_when_blocked() -> None:
    """Test backtracking when occupant cannot vacate, forcing caller to choose alternate."""
    # Create narrow cul-de-sac where (2, 3) only connects to (2, 2)
    # Obstacles surrounding (2, 3)
    obstacles = [(1, 3), (3, 3), (2, 4)]
    grid = GridWorld(10, 10, obstacles=obstacles)
    table = SpaceTimeReservationTable()
    planner = PIBTLocalPlanner(grid, table)

    agent_a = PIBTAgentState(
        'amr_0', current_pos=(2, 2), goal_pos=(2, 3), task_priority=3,
    )
    agent_b = PIBTAgentState(
        'amr_1', current_pos=(2, 3), goal_pos=(2, 3), task_priority=1,
    )

    agents = {'amr_0': agent_a, 'amr_1': agent_b}
    moves, _ = planner.plan_step(agents, time_step=0)

    # amr_1 could not vacate (all exits blocked)
    # amr_0 backtracked and chose an alternative move or waited, leaving (2, 3) to amr_1
    assert moves['amr_1'] == (2, 3)
    assert moves['amr_0'] != (2, 3)


def test_pibt_bitwise_determinism() -> None:
    """Test that repeated runs with identical initial conditions produce identical results."""
    grid = GridWorld.create_warehouse_grid(resolution=0.5)

    def run_simulation() -> List[Position]:
        table = SpaceTimeReservationTable()
        planner = PIBTLocalPlanner(grid, table)
        agents = {
            'amr_0': PIBTAgentState('amr_0', (4, 4), (10, 10), task_priority=2),
            'amr_1': PIBTAgentState('amr_1', (5, 4), (2, 2), task_priority=1),
            'amr_2': PIBTAgentState('amr_2', (4, 5), (6, 6), task_priority=3),
        }
        paths = planner.plan_window(agents, start_time_step=0, window_size=5)
        return paths['amr_0']

    path_run_1 = run_simulation()
    path_run_2 = run_simulation()
    assert path_run_1 == path_run_2


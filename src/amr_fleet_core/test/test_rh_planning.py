"""Unit tests for M5 Rolling-Horizon Single-Agent A* Path Planning."""

from amr_fleet_core.rh_planner import RollingHorizonPlanner, SingleAgentAStar
from amr_fleet_sim.grid_world import GridWorld


def test_astar_straight_line():
    """Test A* on an empty 10x10 grid finds direct Manhattan path."""
    grid = GridWorld(10, 10)
    astar = SingleAgentAStar(grid)

    start = (1, 1)
    goal = (5, 1)
    path = astar.find_path(start, goal)

    assert path is not None
    assert len(path) == 5
    assert path[0] == start
    assert path[-1] == goal


def test_astar_around_obstacle():
    """Test A* navigates around a static obstacle barrier."""
    grid = GridWorld(10, 10)
    # Barrier from (3, 0) to (3, 7)
    for y in range(8):
        grid.add_obstacle((3, y))

    astar = SingleAgentAStar(grid)
    start = (1, 4)
    goal = (5, 4)
    path = astar.find_path(start, goal)

    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    for pos in path:
        assert grid.is_free(pos)


def test_astar_unreachable_goal():
    """Test A* returns None when goal is completely enclosed."""
    grid = GridWorld(10, 10)
    goal = (5, 5)
    # Enclose goal
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        grid.add_obstacle((goal[0] + dx, goal[1] + dy))

    astar = SingleAgentAStar(grid)
    start = (1, 1)
    path = astar.find_path(start, goal)

    assert path is None


def test_astar_start_equals_goal():
    """Test A* when start is identical to goal."""
    grid = GridWorld(10, 10)
    astar = SingleAgentAStar(grid)
    pos = (3, 3)
    path = astar.find_path(pos, pos)

    assert path is not None
    assert path == [pos]


def test_astar_deterministic():
    """Test that multiple runs of A* produce identical paths."""
    grid = GridWorld.create_warehouse_grid(resolution=0.5, warehouse_size=16.0)
    astar = SingleAgentAStar(grid)

    start = (4, 4)
    goal = (26, 26)

    path1 = astar.find_path(start, goal)
    path2 = astar.find_path(start, goal)

    assert path1 is not None
    assert path1 == path2


def test_rh_planner_empty_bundle():
    """Test RollingHorizonPlanner with empty bundle returns success with no sub-goals."""
    planner = RollingHorizonPlanner(robot_id='amr_0')
    planner.update_position((2.0, 2.0))
    planner.update_assigned_bundle([], {})
    response = planner.replan()

    assert response.success is True
    assert response.sub_goal_type == 'NONE'
    assert len(response.full_path) == 0
    assert len(response.horizon_path) == 0
    assert len(response.execution_path) == 0


def test_rh_planner_single_task():
    """Test RollingHorizonPlanner with single task generates pickup sub-goal first."""
    planner = RollingHorizonPlanner(robot_id='amr_0')
    planner.update_position((2.0, 2.0))
    task_dict = {
        'task_id': 'T001',
        'pickup': (2.0, 6.0),
        'dropoff': (8.0, 8.0),
        'priority': 3,
        'deadline': 0.0,
    }
    planner.update_assigned_bundle(['T001'], {'T001': task_dict})

    response = planner.replan()
    assert response.success is True
    assert response.task_id == 'T001'
    assert response.sub_goal_type == 'PICKUP'
    assert len(response.full_path) > 0
    assert len(response.horizon_path) <= planner.config.horizon_steps + 1
    assert len(response.execution_path) <= planner.config.execution_window + 1


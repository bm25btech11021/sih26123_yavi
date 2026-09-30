"""Unit tests for M5 Horizon Truncation, Execution Window, and Replan Triggers."""

from amr_fleet_core.rh_planner import RHConfig, RollingHorizonPlanner


def test_horizon_truncation():
    """Verify that horizon_path has length at most h+1, and execution_path at most w+1."""
    config = RHConfig(horizon_steps=5, execution_window=2)
    planner = RollingHorizonPlanner(robot_id='amr_0', config=config)
    planner.update_position((2.0, 2.0))

    task_dict = {
        'task_id': 'T_LONG',
        'pickup': (14.0, 14.0),
        'dropoff': (2.0, 2.0),
        'priority': 2,
        'deadline': 0.0,
    }
    planner.update_assigned_bundle(['T_LONG'], {'T_LONG': task_dict})

    response = planner.replan()
    assert response.success is True
    assert len(response.full_path) > 6  # Manhattan distance is ~24m, > 6 steps
    assert len(response.horizon_path) == 6  # 5 steps + 1 start
    assert len(response.execution_path) == 3  # 2 steps + 1 start


def test_replan_on_new_bundle():
    """Planner update_assigned_bundle returns True on change and initializes plan."""
    planner = RollingHorizonPlanner(robot_id='amr_0')
    planner.update_position((2.0, 2.0))

    task_dict = {
        'task_id': 'T1',
        'pickup': (2.0, 6.0),
        'dropoff': (8.0, 8.0),
        'priority': 1,
        'deadline': 0.0,
    }
    changed = planner.update_assigned_bundle(['T1'], {'T1': task_dict})
    assert changed is True
    assert planner.active_phase == 'TRANSIT_TO_PICKUP'
    assert planner.current_goal == (2.0, 6.0)
    assert len(planner.execution_path) > 0


def test_replan_on_goal_reached():
    """Planner detects arrival, advances sub-goal, and updates goal from PICKUP to DROPOFF."""
    planner = RollingHorizonPlanner(robot_id='amr_0')
    planner.update_position((2.0, 2.0))
    task_dict = {
        'task_id': 'T1',
        'pickup': (2.0, 6.0),
        'dropoff': (8.0, 8.0),
        'priority': 1,
        'deadline': 0.0,
    }
    planner.update_assigned_bundle(['T1'], {'T1': task_dict})
    assert planner.active_phase == 'TRANSIT_TO_PICKUP'

    # Move to pickup goal (within tolerance 0.5m)
    planner.update_position((2.1, 6.1))
    assert planner.check_subgoal_arrival() is True
    assert planner.check_replan_triggers() is True

    # Advance sub-goal
    event = planner.advance_subgoal()
    assert event == 'PICKUP_REACHED'
    assert planner.active_phase == 'TRANSIT_TO_DROPOFF'
    assert planner.current_goal == (8.0, 8.0)


def test_replan_on_window_expiration():
    """Planner triggers replan after execution window steps are completed."""
    config = RHConfig(horizon_steps=8, execution_window=3)
    planner = RollingHorizonPlanner(robot_id='amr_0', config=config)
    planner.update_position((2.0, 2.0))
    task_dict = {
        'task_id': 'T1',
        'pickup': (13.0, 2.0),
        'dropoff': (8.0, 8.0),
        'priority': 1,
        'deadline': 0.0,
    }
    planner.update_assigned_bundle(['T1'], {'T1': task_dict})
    assert planner.steps_executed_in_window == 0

    # Advance 1 step
    planner.advance_execution_step()
    assert planner.steps_executed_in_window == 1
    assert planner.check_replan_triggers() is False

    # Advance 2nd step
    planner.advance_execution_step()
    assert planner.steps_executed_in_window == 2
    assert planner.check_replan_triggers() is False

    # Advance 3rd step (hits window limit 3)
    planner.advance_execution_step()
    assert planner.steps_executed_in_window == 3
    assert planner.check_replan_triggers() is True


def test_plan_dict_serialization():
    """Planner correctly serializes internal state into a structured dictionary."""
    planner = RollingHorizonPlanner(robot_id='amr_0')
    planner.update_position((2.0, 2.0))
    task_dict = {
        'task_id': 'T1',
        'pickup': (2.0, 6.0),
        'dropoff': (8.0, 8.0),
        'priority': 2,
        'deadline': 0.0,
    }
    planner.update_assigned_bundle(['T1'], {'T1': task_dict})
    plan_dict = planner.to_plan_dict()

    assert plan_dict['robot_id'] == 'amr_0'
    assert plan_dict['current_task_id'] == 'T1'
    assert plan_dict['current_phase'] == 'TRANSIT_TO_PICKUP'
    assert plan_dict['is_valid'] is True
    assert plan_dict['replan_count'] >= 1


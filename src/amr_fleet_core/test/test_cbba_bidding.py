"""Unit tests for CBBA marginal utility scoring and bundle construction."""

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig


def test_cbba_utility_respects_distance():
    """Verify that closer tasks receive higher marginal utility bids."""
    config = CBBAConfig(weight_priority=100.0, weight_distance=10.0)
    agent = CBBAAgent(robot_id='amr_0', config=config, initial_position=(0.0, 0.0))

    near_task = {
        'task_id': 'near',
        'pickup': (1.0, 0.0),
        'dropoff': (2.0, 0.0),
        'priority': 2,
    }
    far_task = {
        'task_id': 'far',
        'pickup': (10.0, 0.0),
        'dropoff': (11.0, 0.0),
        'priority': 2,
    }
    task_map = {'near': near_task, 'far': far_task}

    score_near, _ = agent.compute_marginal_utility(near_task, task_map)
    score_far, _ = agent.compute_marginal_utility(far_task, task_map)

    assert score_near > score_far


def test_cbba_utility_respects_priority():
    """Verify that higher priority tasks receive higher utility for equidistant tasks."""
    config = CBBAConfig(weight_priority=100.0, weight_distance=10.0)
    agent = CBBAAgent(robot_id='amr_0', config=config, initial_position=(0.0, 0.0))

    tasks = {
        'low': {'task_id': 'low', 'pickup': (5.0, 0.0), 'dropoff': (6.0, 0.0), 'priority': 1},
        'norm': {'task_id': 'norm', 'pickup': (5.0, 0.0), 'dropoff': (6.0, 0.0), 'priority': 2},
        'high': {'task_id': 'high', 'pickup': (5.0, 0.0), 'dropoff': (6.0, 0.0), 'priority': 3},
        'crit': {'task_id': 'crit', 'pickup': (5.0, 0.0), 'dropoff': (6.0, 0.0), 'priority': 4},
    }

    score_low, _ = agent.compute_marginal_utility(tasks['low'], tasks)
    score_norm, _ = agent.compute_marginal_utility(tasks['norm'], tasks)
    score_high, _ = agent.compute_marginal_utility(tasks['high'], tasks)
    score_crit, _ = agent.compute_marginal_utility(tasks['crit'], tasks)

    assert score_crit > score_high > score_norm > score_low


def test_cbba_deadline_lateness_penalty():
    """Verify that late tasks receive lateness penalty."""
    config = CBBAConfig(weight_priority=100.0, weight_distance=1.0, weight_late=20.0)
    agent = CBBAAgent(robot_id='amr_0', config=config, initial_position=(0.0, 0.0))

    # Path distance = 10m to pickup + 10m to dropoff = 20m. At 0.5m/s = 40s travel time.
    task_no_deadline = {
        'task_id': 't1',
        'pickup': (10.0, 0.0),
        'dropoff': (20.0, 0.0),
        'priority': 2,
    }
    task_tight_deadline = {
        'task_id': 't2',
        'pickup': (10.0, 0.0),
        'dropoff': (20.0, 0.0),
        'priority': 2,
        'deadline': 10.0,  # Far earlier than 40s arrival
    }
    task_map = {'t1': task_no_deadline, 't2': task_tight_deadline}

    score_normal, _ = agent.compute_marginal_utility(task_no_deadline, task_map)
    score_late, _ = agent.compute_marginal_utility(task_tight_deadline, task_map)

    assert score_normal > score_late


def test_cbba_bundle_capacity_limit():
    """Verify bundle size does not exceed configured capacity."""
    config = CBBAConfig(max_bundle_size=2)
    agent = CBBAAgent(robot_id='amr_0', config=config, initial_position=(0.0, 0.0))

    tasks = {
        f't_{i}': {
            'task_id': f't_{i}',
            'pickup': (float(i), 0.0),
            'dropoff': (float(i) + 0.5, 0.0),
            'priority': 2,
        }
        for i in range(5)
    }

    added = agent.build_bundle(tasks)
    assert added == 2
    assert len(agent.state.bundle) == 2
    assert len(agent.state.path) == 4  # 2 pickups + 2 dropoffs


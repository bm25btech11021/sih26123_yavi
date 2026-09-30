"""Unit tests for CBBA consensus, conflict resolution, and cascade dropping."""

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig


def test_cbba_outbid_and_cascade_drop():
    """Verify that an outbid task and all subsequent tasks in bundle are dropped."""
    config = CBBAConfig(max_bundle_size=4)
    agent = CBBAAgent(robot_id='amr_0', config=config, initial_position=(0.0, 0.0))

    tasks = {
        't1': {'task_id': 't1', 'pickup': (1.0, 0.0), 'dropoff': (2.0, 0.0), 'priority': 2},
        't2': {'task_id': 't2', 'pickup': (3.0, 0.0), 'dropoff': (4.0, 0.0), 'priority': 2},
        't3': {'task_id': 't3', 'pickup': (5.0, 0.0), 'dropoff': (6.0, 0.0), 'priority': 2},
    }

    agent.build_bundle(tasks)
    assert len(agent.state.bundle) == 3
    assert agent.state.bundle == ['t1', 't2', 't3']

    # Peer amr_1 outbids amr_0 on t2
    peer_bids = {'t2': 999.0}
    peer_robots = {'t2': 'amr_1'}
    peer_times = {'t2': 10.0}

    changed = agent.resolve_conflicts(
        peer_id='amr_1',
        peer_iteration=1,
        peer_winning_bids=peer_bids,
        peer_winning_robots=peer_robots,
        peer_timestamps=peer_times,
        task_map=tasks,
    )

    assert changed is True
    # t2 was at index 1 -> t2 and t3 (subsequent) must be dropped!
    assert agent.state.bundle == ['t1']
    assert agent.state.winning_robots['t1'] == 'amr_0'
    assert agent.state.winning_robots['t2'] == 'amr_1'
    assert agent.state.winning_robots['t3'] == ''


def test_cbba_deterministic_tie_breaking():
    """Verify that identical bids break tie in favor of lowest robot_id."""
    config = CBBAConfig(epsilon=1e-6)
    agent_0 = CBBAAgent(robot_id='amr_0', config=config, initial_position=(0.0, 0.0))
    agent_1 = CBBAAgent(robot_id='amr_1', config=config, initial_position=(0.0, 0.0))

    tasks = {
        't1': {'task_id': 't1', 'pickup': (5.0, 0.0), 'dropoff': (6.0, 0.0), 'priority': 2},
    }

    # Both robots build bundle on identical task at identical position
    agent_0.build_bundle(tasks)
    agent_1.build_bundle(tasks)

    assert agent_0.state.bundle == ['t1']
    assert agent_1.state.bundle == ['t1']
    # Identical bids
    assert abs(agent_0.state.winning_bids['t1'] - agent_1.state.winning_bids['t1']) < 1e-6

    # Exchange 1: amr_1 receives amr_0's bid
    changed_1 = agent_1.resolve_conflicts(
        peer_id=agent_0.robot_id,
        peer_iteration=1,
        peer_winning_bids=agent_0.state.winning_bids,
        peer_winning_robots=agent_0.state.winning_robots,
        peer_timestamps=agent_0.state.timestamps,
        task_map=tasks,
    )
    # amr_0 < amr_1 lexicographically, so amr_1 must yield to amr_0
    assert changed_1 is True
    assert agent_1.state.winning_robots['t1'] == 'amr_0'
    assert len(agent_1.state.bundle) == 0

    # Exchange 2: amr_0 receives amr_1's bid
    changed_0 = agent_0.resolve_conflicts(
        peer_id=agent_1.robot_id,
        peer_iteration=1,
        peer_winning_bids=agent_1.state.winning_bids,
        peer_winning_robots=agent_1.state.winning_robots,
        peer_timestamps=agent_1.state.timestamps,
        task_map=tasks,
    )
    # amr_0 was already winner and tie breaks in its favor, so no change
    assert changed_0 is False
    assert agent_0.state.winning_robots['t1'] == 'amr_0'
    assert agent_0.state.bundle == ['t1']


"""Unit tests for Wait-For Graph (WFG) and Deadlock Recovery (M6)."""

from amr_fleet_core.deadlock_recovery import DeadlockRecoveryManager
from amr_fleet_core.pibt_planner import PIBTAgentState
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.wfg_deadlock import DeadlockDetector, WaitForGraph
from amr_fleet_sim.grid_world import GridWorld


def test_wfg_no_cycle() -> None:
    """Test directed acyclic dependency chain (A -> B -> C)."""
    wfg = WaitForGraph()
    wfg.add_wait('amr_0', 'amr_1', resource=(2, 2), time_step=1)
    wfg.add_wait('amr_1', 'amr_2', resource=(3, 2), time_step=1)

    cycles = wfg.find_cycles()
    assert len(cycles) == 0


def test_wfg_two_node_cycle() -> None:
    """Test 2-node head-on cycle (A -> B -> A)."""
    wfg = WaitForGraph()
    wfg.add_wait('amr_0', 'amr_1', resource=(2, 2), time_step=1)
    wfg.add_wait('amr_1', 'amr_0', resource=(2, 3), time_step=1)

    cycles = wfg.find_cycles()
    assert len(cycles) == 1
    assert set(cycles[0]) == {'amr_0', 'amr_1'}


def test_wfg_three_node_cycle() -> None:
    """Test 3-node cyclic waiting loop (A -> B -> C -> A)."""
    wfg = WaitForGraph()
    wfg.add_wait('amr_0', 'amr_1', resource=(2, 2), time_step=1)
    wfg.add_wait('amr_1', 'amr_2', resource=(3, 3), time_step=1)
    wfg.add_wait('amr_2', 'amr_0', resource=(1, 1), time_step=1)

    cycles = wfg.find_cycles()
    assert len(cycles) == 1
    assert set(cycles[0]) == {'amr_0', 'amr_1', 'amr_2'}


def test_transient_wait_not_deadlock() -> None:
    """Test that a short temporary wait does not trigger false-positive deadlock."""
    detector = DeadlockDetector(persistence_threshold_sec=2.0, min_stall_cycles=4)
    wfg = WaitForGraph()
    wfg.add_wait('amr_0', 'amr_1', resource=(2, 2), time_step=1)
    wfg.add_wait('amr_1', 'amr_0', resource=(2, 3), time_step=1)

    positions = {'amr_0': (2.0, 2.0), 'amr_1': (2.0, 3.0)}

    # Step 1 at t=0.0: first observation
    deadlocks = detector.update(wfg, positions, now_sec=0.0)
    assert len(deadlocks) == 0

    # Step 2 at t=0.5 (< 2.0s and counts=2 < 4): still transient
    deadlocks = detector.update(wfg, positions, now_sec=0.5)
    assert len(deadlocks) == 0


def test_persistent_deadlock_detection() -> None:
    """Test that a stalled cycle exceeding threshold triggers confirmed deadlock."""
    detector = DeadlockDetector(persistence_threshold_sec=1.0, min_stall_cycles=3)
    wfg = WaitForGraph()
    wfg.add_wait('amr_0', 'amr_1', resource=(2, 2), time_step=1)
    wfg.add_wait('amr_1', 'amr_0', resource=(2, 3), time_step=1)

    positions = {'amr_0': (2.0, 2.0), 'amr_1': (2.0, 3.0)}

    # Cycle observed across multiple stalled timesteps
    detector.update(wfg, positions, now_sec=0.0)
    detector.update(wfg, positions, now_sec=0.5)
    detector.update(wfg, positions, now_sec=1.1)
    deadlocks = detector.update(wfg, positions, now_sec=1.5)

    assert len(deadlocks) == 1
    dl = deadlocks[0]
    assert set(dl.cycle_robot_ids) == {'amr_0', 'amr_1'}
    assert dl.persistence_duration_sec >= 1.0


def test_deadlock_recovery_sidestep() -> None:
    """Test deterministic recovery via victim selection and lateral escape move."""
    grid = GridWorld(10, 10)
    table = SpaceTimeReservationTable()
    recovery = DeadlockRecoveryManager(grid, table)

    # amr_0 (priority 3) and amr_1 (priority 1) facing each other at (2, 2) and (2, 3)
    agents = {
        'amr_0': PIBTAgentState('amr_0', current_pos=(2, 2), goal_pos=(2, 4), task_priority=3),
        'amr_1': PIBTAgentState('amr_1', current_pos=(2, 3), goal_pos=(2, 1), task_priority=1),
    }

    # amr_1 has lowest priority -> should be chosen as victim
    victim = recovery.select_victim(['amr_0', 'amr_1'], agents)
    assert victim == 'amr_1'

    # Lateral escape cell found for amr_1 (e.g. (1, 3) or (3, 3))
    escape = recovery.find_escape_cell('amr_1', agents, ['amr_0', 'amr_1'])
    assert escape is not None
    assert escape in {(1, 3), (3, 3)}

    # Execute recovery action
    from amr_fleet_core.coordination_models import DeadlockRecord
    rec = DeadlockRecord(
        cycle_robot_ids=['amr_0', 'amr_1'],
        root_cause='Cycle',
        persistence_duration_sec=2.0,
        recovery_action='',
    )
    success = recovery.execute_recovery(rec, agents, time_step=5)
    assert success is True
    assert rec.recovery_success is True
    assert 'SIDESTEP' in rec.recovery_action
    assert agents['amr_1'].planned_moves[6] == escape


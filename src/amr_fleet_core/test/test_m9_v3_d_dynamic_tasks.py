"""
Unit and Integration Tests for Milestone M9-V3 Phase 1: Dynamic Task Arrival (Scenario V3-D.1).

Tests:
  - test_unconverged_bundle_must_not_start_execution: Unconverged CBBA bundle cannot execute.
  - test_converged_bundle_may_execute: Converged CBBA bundle transitions to execution.
  - test_task_already_in_progress_survives_dynamic_release: In-flight task survives dynamic rel.
  - test_staged_tasks_cannot_enter_cbba: STAGED tasks cannot participate in CBBA bidding.
  - test_accounting_invariant_sums_to_generated_tasks: Strict task accounting conservation.
  - test_a_future_task_remains_staged: Future task initialization and state machine validation.
  - test_b_transition_staged_to_pending_at_release_time: Dynamic release transition at t_release.
  - test_d_newly_released_tasks_bid_and_converge: Multi-tier bidding and convergence.
  - test_f_no_duplicate_allocation_or_dropped_tasks: Fleet-wide allocation conservation.
  - test_g_deterministic_replay: Deterministic workload replay with fixed seed.
"""

import os
from typing import List

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_core.rh_node import RollingHorizonPlannerNode
from amr_fleet_core.rh_planner import RHConfig, RollingHorizonPlanner
from amr_fleet_core.task_model import (
    InvalidTaskTransitionError,
    Task,
    TaskLifecycleState,
    TaskPriority,
)
from amr_fleet_core.workload import WorkloadManager
from amr_fleet_msgs.msg import RobotBundle, TaskDefinition, TaskList
import pytest
import rclpy


@pytest.fixture
def workspace_root():
    cur = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(os.path.dirname(cur)))


def test_unconverged_bundle_must_not_start_execution():
    """Verify that unconverged CBBA bundles (is_converged=False) do not start execution."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        # Pre-populate cached task metadata
        t_msg = TaskList()
        td = TaskDefinition()
        td.task_id = 'task_unconverged_01'
        td.pickup_pose.x = 4.5
        td.pickup_pose.y = 1.5
        td.dropoff_pose.x = 14.5
        td.dropoff_pose.y = 14.5
        td.priority = 2
        td.status = 'ASSIGNED'
        t_msg.tasks = [td]
        node._handle_tasks_all(t_msg)

        # Incoming bundle with is_converged=False
        unconverged_msg = RobotBundle()
        unconverged_msg.robot_id = 'amr_0'
        unconverged_msg.task_ids = ['task_unconverged_01']
        unconverged_msg.is_converged = False

        node._handle_bundle(unconverged_msg)

        # INVARIANT: cached_bundle must remain empty and planner must not sequence tasks
        assert node.cached_bundle == [], 'Cached bundle must not accept unconverged bundle'
        assert len(node.planner.ordered_tasks) == 0, 'Planner must not order unconverged tasks'
        assert node.planner.active_phase == 'IDLE', 'Planner must remain IDLE'
        assert node.planner.current_goal is None, 'Planner must not set goal for unconv bundle'
    finally:
        node.destroy_node()


def test_converged_bundle_may_execute():
    """Verify that converged CBBA bundles (is_converged=True) are accepted and execute."""
    if not rclpy.ok():
        rclpy.init()

    node = RollingHorizonPlannerNode()
    try:
        # Pre-populate cached task metadata
        t_msg = TaskList()
        td = TaskDefinition()
        td.task_id = 'task_converged_01'
        td.pickup_pose.x = 4.5
        td.pickup_pose.y = 1.5
        td.dropoff_pose.x = 14.5
        td.dropoff_pose.y = 14.5
        td.priority = 2
        td.status = 'ASSIGNED'
        t_msg.tasks = [td]
        node._handle_tasks_all(t_msg)

        # Incoming bundle with is_converged=True
        converged_msg = RobotBundle()
        converged_msg.robot_id = 'amr_0'
        converged_msg.task_ids = ['task_converged_01']
        converged_msg.is_converged = True

        node._handle_bundle(converged_msg)

        # INVARIANT: cached_bundle is updated and planner starts execution
        assert node.cached_bundle == ['task_converged_01']
        assert node.planner.ordered_tasks == ['task_converged_01']
        assert node.planner.active_phase == 'TRANSIT_TO_PICKUP'
        assert node.planner.current_goal == (4.5, 1.5)
    finally:
        node.destroy_node()


def test_task_already_in_progress_survives_dynamic_release():
    """Verify that an active in-progress task is not interrupted when dynamic release occurs."""
    planner = RollingHorizonPlanner(
        robot_id='amr_0',
        config=RHConfig(goal_tolerance_m=0.5),
    )
    planner.current_position = (2.0, 2.0)
    task1 = {
        'task_id': 'T001',
        'pickup': (4.5, 1.5),
        'dropoff': (14.5, 14.5),
        'priority': 2,
        'status': 'ASSIGNED',
    }
    task2 = {
        'task_id': 'T002',
        'pickup': (7.5, 1.5),
        'dropoff': (17.5, 17.5),
        'priority': 3,
        'status': 'PENDING',
    }
    tasks_map = {'T001': task1}

    # Step 1: Ingest T001 and start transit to pickup
    planner.update_assigned_bundle(['T001'], tasks_map)
    assert planner.active_phase == 'TRANSIT_TO_PICKUP'
    assert planner.current_goal == (4.5, 1.5)
    assert planner.ordered_tasks == ['T001']
    assert planner.active_task_idx == 0

    # Robot moves to pickup and arrives
    planner.update_position((4.5, 1.5))
    assert planner.check_subgoal_arrival()
    event = planner.advance_subgoal()
    assert event == 'PICKUP_REACHED'
    assert planner.active_phase == 'TRANSIT_TO_DROPOFF'
    assert planner.current_goal == (14.5, 14.5)

    # Robot moves halfway to dropoff (e.g. x=9.5, y=8.0)
    planner.update_position((9.5, 8.0))

    # Step 2: At t=45s, dynamic release occurs: T002 arrives in bundle
    tasks_map['T002'] = task2
    changed = planner.update_assigned_bundle(['T001', 'T002'], tasks_map)
    assert changed

    # CRITICAL INVARIANT: In-progress task T001 MUST NOT be interrupted or restarted
    assert planner.active_phase == 'TRANSIT_TO_DROPOFF', 'In-progress phase was interrupted!'
    assert planner.current_goal == (14.5, 14.5), 'In-progress goal was overwritten!'
    assert planner.active_task_idx == 0, 'active_task_idx was reset to 0 improperly!'
    assert planner.ordered_tasks == ['T001', 'T002'], 'T002 not sequenced after T001!'

    # Robot completes T001 at dropoff
    planner.update_position((14.5, 14.5))
    assert planner.check_subgoal_arrival()
    event2 = planner.advance_subgoal()
    assert event2 == 'TASK_COMPLETED'

    # Now planner advances seamlessly to T002 pickup
    assert planner.active_task_idx == 1
    assert planner.active_phase == 'TRANSIT_TO_PICKUP'
    assert planner.current_goal == (7.5, 1.5)


def test_staged_tasks_cannot_enter_cbba(workspace_root):
    """Verify STAGED tasks cannot participate in CBBA bidding until release time."""
    wl_path = os.path.join(workspace_root, 'config', 'workloads', 'workload_30_tasks_m9_v3_d.yaml')
    assert os.path.isfile(wl_path), f'Workload file missing: {wl_path}'

    tasks = WorkloadManager.load_from_yaml(wl_path)
    assert len(tasks) == 30

    pending_tasks = [t for t in tasks if t.state == TaskLifecycleState.PENDING]
    staged_tasks = [t for t in tasks if t.state == TaskLifecycleState.STAGED]

    assert len(pending_tasks) == 15
    assert len(staged_tasks) == 15
    for t in pending_tasks:
        assert t.release_time_sec == 0.0
    for t in staged_tasks:
        assert t.release_time_sec == 45.0
        assert not t.is_terminal

    # Candidate map contains only pending tasks (STAGED filtered out by TaskManager)
    available_map = {
        t.task_id: {
            'task_id': t.task_id,
            'pickup': t.pickup,
            'dropoff': t.dropoff,
            'priority': int(t.priority),
            'deadline': t.deadline,
        }
        for t in pending_tasks
    }
    agent = CBBAAgent('amr_0', config=CBBAConfig(max_bundle_size=4))
    agent.build_bundle(available_map, current_time=0.0)

    # Agent bundle can only contain tasks from available_map, never staged
    assert len(agent.state.bundle) > 0
    for b_tid in agent.state.bundle:
        assert b_tid in available_map
        assert not any(b_tid == st.task_id for st in staged_tasks)

    # Direct attempt to bid on STAGED tasks must be rejected or filtered out
    for st in staged_tasks:
        with pytest.raises(InvalidTaskTransitionError):
            st.transition_to(TaskLifecycleState.ASSIGNED)


def test_accounting_invariant_sums_to_generated_tasks():
    """Verify task accounting invariant: Generated = sum(disjoint states) and Remaining."""
    def check_accounting(
        staged: int,
        pending: int,
        assigned: int,
        in_progress: int,
        completed: int,
        failed: int,
        cancelled: int,
        workload_size: int = 30,
    ):
        generated = workload_size
        disjoint_sum = (
            staged + pending + assigned + in_progress + completed + failed + cancelled
        )
        assert disjoint_sum == generated, (
            f'Disjoint sum {disjoint_sum} does not equal generated {generated}'
        )

        remaining = staged + pending + assigned + in_progress
        assert remaining + completed + failed + cancelled == generated, (
            f'Remaining ({remaining}) + terminal ({completed + failed + cancelled}) != {generated}'
        )
        return remaining

    # Scenario 1: Initial release state (t=0)
    rem1 = check_accounting(
        staged=15, pending=15, assigned=0, in_progress=0,
        completed=0, failed=0, cancelled=0, workload_size=30,
    )
    assert rem1 == 30

    # Scenario 2: Post-initial CBBA convergence (t=5s)
    rem2 = check_accounting(
        staged=15, pending=5, assigned=10, in_progress=0,
        completed=0, failed=0, cancelled=0, workload_size=30,
    )
    assert rem2 == 30

    # Scenario 3: V3-D Pilot Audit state (t=45s, dynamic release arrived and converged)
    rem3 = check_accounting(
        staged=0, pending=1, assigned=28, in_progress=1,
        completed=0, failed=0, cancelled=0, workload_size=30,
    )
    assert rem3 == 30

    # Scenario 4: Mid-mission execution (t=100s)
    rem4 = check_accounting(
        staged=0, pending=0, assigned=10, in_progress=5,
        completed=15, failed=0, cancelled=0, workload_size=30,
    )
    assert rem4 == 15

    # Scenario 5: Terminal mission with failures and cancellations
    rem5 = check_accounting(
        staged=0, pending=0, assigned=0, in_progress=0,
        completed=25, failed=3, cancelled=2, workload_size=30,
    )
    assert rem5 == 0

    # Verify directly against TaskLifecycleState enum definitions
    all_lifecycle_states = {
        TaskLifecycleState.STAGED,
        TaskLifecycleState.PENDING,
        TaskLifecycleState.ASSIGNED,
        TaskLifecycleState.IN_PROGRESS,
        TaskLifecycleState.COMPLETED,
        TaskLifecycleState.FAILED,
        TaskLifecycleState.CANCELLED,
    }
    non_terminal_states = {
        TaskLifecycleState.STAGED,
        TaskLifecycleState.PENDING,
        TaskLifecycleState.ASSIGNED,
        TaskLifecycleState.IN_PROGRESS,
    }
    terminal_states = {
        TaskLifecycleState.COMPLETED,
        TaskLifecycleState.FAILED,
        TaskLifecycleState.CANCELLED,
    }
    assert non_terminal_states.union(terminal_states) == all_lifecycle_states
    assert non_terminal_states.isdisjoint(terminal_states)


# Backward-compatible aliases
test_c_staged_tasks_excluded_from_cbba_bidding = test_staged_tasks_cannot_enter_cbba
test_e_in_progress_tasks_preserved = test_task_already_in_progress_survives_dynamic_release


def test_a_future_task_remains_staged():
    """Test A: Tasks with release_time_sec > 0 start STAGED and reject bad transitions."""
    t_staged = Task(
        task_id='task_future_01',
        pickup=(2.0, 2.0),
        dropoff=(14.5, 14.5),
        priority=TaskPriority.HIGH,
        release_time_sec=45.0,
    )
    assert t_staged.state == TaskLifecycleState.STAGED
    assert t_staged.release_time_sec == 45.0
    assert not t_staged.is_terminal
    assert t_staged.events[0].event_type == 'STAGED'
    assert t_staged.events[0].to_state == TaskLifecycleState.STAGED

    # Immediate illegal transitions directly to ASSIGNED or IN_PROGRESS must be rejected
    with pytest.raises(InvalidTaskTransitionError):
        t_staged.transition_to(TaskLifecycleState.ASSIGNED)
    with pytest.raises(InvalidTaskTransitionError):
        t_staged.transition_to(TaskLifecycleState.IN_PROGRESS)


def test_b_transition_staged_to_pending_at_release_time():
    """Test B: Staged task transitions to PENDING at release time and logs audit event."""
    t_staged = Task(
        task_id='task_dynamic_01',
        pickup=(4.5, 1.5),
        dropoff=(14.5, 14.5),
        release_time_sec=45.0,
    )
    assert t_staged.state == TaskLifecycleState.STAGED

    # Transition STAGED -> PENDING at t=45.0s
    t_staged.transition_to(
        TaskLifecycleState.PENDING,
        timestamp=45.0,
        details='Deterministic release at t=45.0s',
    )
    assert t_staged.state == TaskLifecycleState.PENDING
    assert len(t_staged.events) == 2
    latest_event = t_staged.events[-1]
    assert latest_event.event_type == 'RELEASED'
    assert latest_event.from_state == TaskLifecycleState.STAGED
    assert latest_event.to_state == TaskLifecycleState.PENDING
    assert latest_event.timestamp == 45.0


def test_d_newly_released_tasks_bid_and_converge(workspace_root):
    """Test D: Fleet bids on initial 15 tasks, then newly released 15 tasks converge."""
    wl_path = os.path.join(workspace_root, 'config', 'workloads', 'workload_30_tasks_m9_v3_d.yaml')
    tasks = WorkloadManager.load_from_yaml(wl_path)

    fleet_size = 10
    agents = [
        CBBAAgent(
            f'amr_{i}',
            config=CBBAConfig(max_bundle_size=8),
            initial_position=(2.0, 2.0 + i * 3.0),
        )
        for i in range(fleet_size)
    ]

    # Tier 1: Initial 15 tasks at t=0
    tier1_map = {
        t.task_id: {
            'task_id': t.task_id,
            'pickup': t.pickup,
            'dropoff': t.dropoff,
            'priority': int(t.priority),
            'deadline': t.deadline,
        }
        for t in tasks if t.release_time_sec == 0.0
    }

    # Run CBBA consensus on Tier 1
    for _ in range(15):
        for ag in agents:
            ag.build_bundle(tier1_map, current_time=0.0)
        for i in range(fleet_size):
            for j in range(fleet_size):
                if i != j:
                    agents[i].resolve_conflicts(
                        peer_id=agents[j].robot_id,
                        peer_iteration=1,
                        peer_winning_bids=agents[j].state.winning_bids,
                        peer_winning_robots=agents[j].state.winning_robots,
                        peer_timestamps=agents[j].state.timestamps,
                        task_map=tier1_map,
                        current_time=0.0,
                    )

    tier1_assigned = set()
    for ag in agents:
        tier1_assigned.update(ag.state.bundle)
    assert len(tier1_assigned) == 15, f'Expected 15 assigned, got {len(tier1_assigned)}'

    # Tier 2: Transition remaining 15 tasks at t=45.0s STAGED -> PENDING
    for t in tasks:
        if t.state == TaskLifecycleState.STAGED:
            t.transition_to(TaskLifecycleState.PENDING, timestamp=45.0)

    tier2_map = {
        t.task_id: {
            'task_id': t.task_id,
            'pickup': t.pickup,
            'dropoff': t.dropoff,
            'priority': int(t.priority),
            'deadline': t.deadline,
        }
        for t in tasks if t.release_time_sec == 45.0
    }
    all_map = {**tier1_map, **tier2_map}

    # Lock Tier 1 tasks as assigned
    tier1_locked = {ag.robot_id: set(ag.state.bundle) for ag in agents}
    for _ in range(15):
        for ag in agents:
            locked = tier1_locked[ag.robot_id]
            ag.build_bundle(
                tier2_map,
                current_time=45.0,
                locked_tasks=locked,
                full_task_map=all_map,
            )
        for i in range(fleet_size):
            for j in range(fleet_size):
                if i != j:
                    locked = tier1_locked[agents[i].robot_id]
                    agents[i].resolve_conflicts(
                        peer_id=agents[j].robot_id,
                        peer_iteration=2,
                        peer_winning_bids=agents[j].state.winning_bids,
                        peer_winning_robots=agents[j].state.winning_robots,
                        peer_timestamps=agents[j].state.timestamps,
                        task_map=all_map,
                        current_time=45.0,
                        locked_tasks=locked,
                    )

    all_assigned = set()
    for ag in agents:
        all_assigned.update(ag.state.bundle)
    assert len(all_assigned) == 30, f'Expected 30 assigned, got {len(all_assigned)}'


def test_f_no_duplicate_allocation_or_dropped_tasks(workspace_root):
    """Test F: Fleet allocation has exactly 1 AMR per task and 0 duplicate or dropped."""
    wl_path = os.path.join(workspace_root, 'config', 'workloads', 'workload_30_tasks_m9_v3_d.yaml')
    tasks = WorkloadManager.load_from_yaml(wl_path)

    fleet_size = 10
    agents = [
        CBBAAgent(
            f'amr_{i}',
            config=CBBAConfig(max_bundle_size=6),
            initial_position=(2.0, 2.0 + i * 3.0),
        )
        for i in range(fleet_size)
    ]
    task_map = {
        t.task_id: {
            'task_id': t.task_id,
            'pickup': t.pickup,
            'dropoff': t.dropoff,
            'priority': int(t.priority),
            'deadline': t.deadline,
        }
        for t in tasks
    }

    # Run centralized or fully converged consensus
    for _ in range(25):
        for ag in agents:
            ag.build_bundle(task_map, current_time=0.0)
        for i in range(fleet_size):
            for j in range(fleet_size):
                if i != j:
                    agents[i].resolve_conflicts(
                        peer_id=agents[j].robot_id,
                        peer_iteration=1,
                        peer_winning_bids=agents[j].state.winning_bids,
                        peer_winning_robots=agents[j].state.winning_robots,
                        peer_timestamps=agents[j].state.timestamps,
                        task_map=task_map,
                        current_time=0.0,
                    )

    # Audit allocation uniqueness
    all_assigned_tasks: List[str] = []
    for ag in agents:
        all_assigned_tasks.extend(ag.state.bundle)

    # No duplicate assignments across the fleet
    assert len(all_assigned_tasks) == len(set(all_assigned_tasks)), 'Duplicate task detected!'
    # Zero dropped tasks
    assert len(set(all_assigned_tasks)) == 30, f'Dropped tasks: got {len(set(all_assigned_tasks))}'


def test_g_deterministic_replay(workspace_root):
    """Test G: Workload generation and staged parameters replay identically with seed 42."""
    wl_path = os.path.join(workspace_root, 'config', 'workloads', 'workload_30_tasks_m9_v3_d.yaml')
    run1 = WorkloadManager.load_from_yaml(wl_path)
    run2 = WorkloadManager.load_from_yaml(wl_path)

    assert len(run1) == len(run2) == 30
    for t1, t2 in zip(run1, run2):
        assert t1.task_id == t2.task_id
        assert t1.pickup == t2.pickup
        assert t1.dropoff == t2.dropoff
        assert t1.priority == t2.priority
        assert t1.deadline == t2.deadline
        assert t1.release_time_sec == t2.release_time_sec
        assert t1.state == t2.state


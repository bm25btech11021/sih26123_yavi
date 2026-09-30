"""Unit tests for task data model and lifecycle state machine."""

from amr_fleet_core.task_model import (
    InvalidTaskTransitionError,
    Task,
    TaskLifecycleState,
    TaskPriority,
)
import pytest


def test_task_initialization():
    """Verify clean task construction with default and custom values."""
    task = Task(
        task_id='task_0001',
        pickup=(2.0, 2.0),
        dropoff=(8.0, 8.0),
        priority=TaskPriority.HIGH,
        created_at=100.0,
        deadline=160.0,
    )
    assert task.task_id == 'task_0001'
    assert task.pickup == (2.0, 2.0)
    assert task.dropoff == (8.0, 8.0)
    assert task.priority == TaskPriority.HIGH
    assert task.created_at == 100.0
    assert task.deadline == 160.0
    assert task.state == TaskLifecycleState.PENDING
    assert not task.is_terminal
    assert task.assigned_robot_id is None
    assert len(task.events) == 1
    assert task.events[0].event_type == 'CREATED'
    assert task.waiting_time is None
    assert task.execution_time is None
    assert task.total_duration is None
    assert task.deadline_slack is None


def test_valid_lifecycle_nominal_flow():
    """Verify nominal lifecycle: PENDING -> ASSIGNED -> IN_PROGRESS -> COMPLETED."""
    task = Task('task_0002', (2.0, 13.0), (7.0, 7.0), created_at=10.0, deadline=60.0)

    # Transition to ASSIGNED
    task.transition_to(TaskLifecycleState.ASSIGNED, timestamp=15.0, robot_id='amr_0')
    assert task.state == TaskLifecycleState.ASSIGNED
    assert task.assigned_robot_id == 'amr_0'
    assert task.assigned_at == 15.0
    assert task.waiting_time == 5.0  # 15.0 - 10.0

    # Transition to IN_PROGRESS
    task.transition_to(TaskLifecycleState.IN_PROGRESS, timestamp=18.0)
    assert task.state == TaskLifecycleState.IN_PROGRESS
    assert task.started_at == 18.0
    assert task.waiting_time == 8.0  # 18.0 - 10.0

    # Transition to COMPLETED
    task.transition_to(TaskLifecycleState.COMPLETED, timestamp=40.0)
    assert task.state == TaskLifecycleState.COMPLETED
    assert task.is_terminal
    assert task.completed_at == 40.0
    assert task.waiting_time == 8.0
    assert task.execution_time == 22.0   # 40.0 - 18.0
    assert task.total_duration == 30.0   # 40.0 - 10.0
    assert task.deadline_slack == 20.0   # 60.0 - 40.0 (on-time)

    # Check event history
    assert len(task.events) == 4
    event_types = [e.event_type for e in task.events]
    assert event_types == ['CREATED', 'ASSIGNED', 'STARTED', 'COMPLETED']


def test_valid_cancellation_and_failure_paths():
    """Verify cancellation and failure transitions from different states."""
    # 1. PENDING -> CANCELLED
    t1 = Task('t1', (2.0, 2.0), (8.0, 8.0), created_at=0.0)
    t1.transition_to(TaskLifecycleState.CANCELLED, timestamp=5.0)
    assert t1.state == TaskLifecycleState.CANCELLED
    assert t1.is_terminal
    assert t1.total_duration == 5.0

    # 2. ASSIGNED -> CANCELLED
    t2 = Task('t2', (2.0, 2.0), (8.0, 8.0), created_at=0.0)
    t2.transition_to(TaskLifecycleState.ASSIGNED, timestamp=2.0, robot_id='amr_1')
    t2.transition_to(TaskLifecycleState.CANCELLED, timestamp=6.0)
    assert t2.state == TaskLifecycleState.CANCELLED
    assert t2.total_duration == 6.0

    # 3. ASSIGNED -> FAILED
    t3 = Task('t3', (2.0, 2.0), (8.0, 8.0), created_at=0.0)
    t3.transition_to(TaskLifecycleState.ASSIGNED, timestamp=2.0, robot_id='amr_1')
    t3.transition_to(TaskLifecycleState.FAILED, timestamp=8.0, details='Hardware failure')
    assert t3.state == TaskLifecycleState.FAILED
    assert t3.total_duration == 8.0

    # 4. IN_PROGRESS -> FAILED
    t4 = Task('t4', (2.0, 2.0), (8.0, 8.0), created_at=0.0)
    t4.transition_to(TaskLifecycleState.ASSIGNED, timestamp=2.0, robot_id='amr_2')
    t4.transition_to(TaskLifecycleState.IN_PROGRESS, timestamp=4.0)
    t4.transition_to(TaskLifecycleState.FAILED, timestamp=15.0, details='Path blocked')
    assert t4.state == TaskLifecycleState.FAILED
    assert t4.execution_time == 11.0  # 15.0 - 4.0

    # 5. IN_PROGRESS -> CANCELLED
    t5 = Task('t5', (2.0, 2.0), (8.0, 8.0), created_at=0.0)
    t5.transition_to(TaskLifecycleState.ASSIGNED, timestamp=2.0, robot_id='amr_2')
    t5.transition_to(TaskLifecycleState.IN_PROGRESS, timestamp=4.0)
    t5.transition_to(TaskLifecycleState.CANCELLED, timestamp=10.0)
    assert t5.state == TaskLifecycleState.CANCELLED
    assert t5.execution_time == 6.0  # 10.0 - 4.0


def test_task_preemption_unassign():
    """Verify ASSIGNED -> PENDING preemption/task release."""
    task = Task('t_preempt', (2.0, 2.0), (8.0, 8.0), created_at=0.0)
    task.transition_to(TaskLifecycleState.ASSIGNED, timestamp=5.0, robot_id='amr_0')
    assert task.assigned_robot_id == 'amr_0'

    # Unassign back to PENDING
    task.transition_to(TaskLifecycleState.PENDING, timestamp=10.0, details='Bid timeout')
    assert task.state == TaskLifecycleState.PENDING
    assert task.assigned_robot_id is None
    assert task.events[-1].event_type == 'UNASSIGNED'

    # Can now be assigned to another robot
    task.transition_to(TaskLifecycleState.ASSIGNED, timestamp=12.0, robot_id='amr_1')
    assert task.state == TaskLifecycleState.ASSIGNED
    assert task.assigned_robot_id == 'amr_1'


def test_invalid_state_transitions():
    """Verify invalid state transitions raise InvalidTaskTransitionError."""
    task = Task('t_invalid', (2.0, 2.0), (8.0, 8.0))

    # Cannot skip from PENDING directly to IN_PROGRESS
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.IN_PROGRESS)

    # Cannot skip from PENDING directly to COMPLETED
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.COMPLETED)

    # Cannot fail while pending
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.FAILED)

    # Complete a task, then test terminal state invariance
    task.transition_to(TaskLifecycleState.ASSIGNED, robot_id='amr_0')
    task.transition_to(TaskLifecycleState.IN_PROGRESS)
    task.transition_to(TaskLifecycleState.COMPLETED)

    # Terminal state transitions must all fail
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.PENDING)
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.ASSIGNED)
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.IN_PROGRESS)
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.FAILED)
    with pytest.raises(InvalidTaskTransitionError):
        task.transition_to(TaskLifecycleState.CANCELLED)


def test_task_serialization_roundtrip():
    """Verify to_dict and from_dict preserve all properties and states."""
    task = Task(
        task_id='t_roundtrip',
        pickup=(2.5, 3.5),
        dropoff=(11.0, 12.0),
        priority=TaskPriority.CRITICAL,
        created_at=50.0,
        deadline=120.0,
        metadata={'batch': 'A1', 'weight_kg': 25.0},
    )
    task.transition_to(TaskLifecycleState.ASSIGNED, timestamp=55.0, robot_id='amr_3')
    task.transition_to(TaskLifecycleState.IN_PROGRESS, timestamp=60.0)

    d = task.to_dict()
    assert d['task_id'] == 't_roundtrip'
    assert d['pickup'] == [2.5, 3.5]
    assert d['dropoff'] == [11.0, 12.0]
    assert d['priority'] == 'CRITICAL'
    assert d['priority_val'] == 4
    assert d['status'] == 'IN_PROGRESS'
    assert d['assigned_robot_id'] == 'amr_3'
    assert d['waiting_time'] == 10.0  # 60.0 - 50.0

    restored = Task.from_dict(d)
    assert restored.task_id == task.task_id
    assert restored.pickup == task.pickup
    assert restored.dropoff == task.dropoff
    assert restored.priority == task.priority
    assert restored.state == task.state
    assert restored.assigned_robot_id == task.assigned_robot_id
    assert restored.metadata == task.metadata


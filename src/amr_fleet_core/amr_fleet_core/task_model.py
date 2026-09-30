"""Task data model and lifecycle state machine for AMR fleet coordination."""

from dataclasses import dataclass
from enum import Enum, IntEnum
import time
from typing import Any, Dict, List, Optional, Set, Tuple


class TaskPriority(IntEnum):
    """Priority levels for task scheduling and allocation."""

    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4

    @classmethod
    def from_str(cls, val: str) -> 'TaskPriority':
        """Convert string to TaskPriority with case-insensitivity."""
        upper = val.strip().upper()
        if upper in cls.__members__:
            return cls[upper]
        try:
            return cls(int(val))
        except (ValueError, KeyError):
            return cls.NORMAL


class TaskLifecycleState(Enum):
    """Discrete lifecycle states for AMR fleet tasks."""

    STAGED = 'STAGED'
    PENDING = 'PENDING'
    ASSIGNED = 'ASSIGNED'
    IN_PROGRESS = 'IN_PROGRESS'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'
    CANCELLED = 'CANCELLED'

    @classmethod
    def from_str(cls, val: str) -> 'TaskLifecycleState':
        """Convert string to TaskLifecycleState."""
        upper = val.strip().upper()
        if upper in cls.__members__:
            return cls[upper]
        raise ValueError(f'Unknown task lifecycle state: {val}')


class InvalidTaskTransitionError(Exception):
    """Raised when an illegal task lifecycle state transition is attempted."""

    def __init__(
        self,
        from_state: TaskLifecycleState,
        to_state: TaskLifecycleState,
        task_id: str = '',
    ):
        msg = f'Invalid task transition from {from_state.value} to {to_state.value}'
        if task_id:
            msg += f" for task '{task_id}'"
        super().__init__(msg)
        self.from_state = from_state
        self.to_state = to_state
        self.task_id = task_id


@dataclass
class TaskEvent:
    """Historical audit record for a single task lifecycle transition."""

    timestamp: float
    event_type: str
    from_state: Optional[TaskLifecycleState]
    to_state: TaskLifecycleState
    robot_id: Optional[str] = None
    details: str = ''

    def to_dict(self) -> Dict[str, Any]:
        """Serialize event to a dictionary."""
        return {
            'timestamp': self.timestamp,
            'event_type': self.event_type,
            'from_state': self.from_state.value if self.from_state else None,
            'to_state': self.to_state.value,
            'robot_id': self.robot_id,
            'details': self.details,
        }


class Task:
    """Formal domain model for a single pickup-and-delivery logistics task."""

    VALID_TRANSITIONS: Dict[TaskLifecycleState, Set[TaskLifecycleState]] = {
        TaskLifecycleState.STAGED: {
            TaskLifecycleState.PENDING,
            TaskLifecycleState.CANCELLED,
        },
        TaskLifecycleState.PENDING: {
            TaskLifecycleState.ASSIGNED,
            TaskLifecycleState.CANCELLED,
        },
        TaskLifecycleState.ASSIGNED: {
            TaskLifecycleState.IN_PROGRESS,
            TaskLifecycleState.PENDING,     # Task preemption / release / requeue
            TaskLifecycleState.CANCELLED,
            TaskLifecycleState.FAILED,
        },
        TaskLifecycleState.IN_PROGRESS: {
            TaskLifecycleState.COMPLETED,
            TaskLifecycleState.FAILED,
            TaskLifecycleState.CANCELLED,
            TaskLifecycleState.PENDING,     # Safe operator abort / requeue
        },
        TaskLifecycleState.COMPLETED: set(),  # Terminal
        TaskLifecycleState.FAILED: {
            TaskLifecycleState.PENDING,     # Operator requeue
        },
        TaskLifecycleState.CANCELLED: set(),  # Terminal
    }

    def __init__(
        self,
        task_id: str,
        pickup: Tuple[float, float],
        dropoff: Tuple[float, float],
        priority: TaskPriority = TaskPriority.NORMAL,
        created_at: Optional[float] = None,
        deadline: Optional[float] = None,
        metadata: Optional[Dict[str, Any]] = None,
        release_time_sec: float = 0.0,
        requested_robot: Optional[str] = None,
    ) -> None:
        self.task_id = task_id
        self.pickup = (float(pickup[0]), float(pickup[1]))
        self.dropoff = (float(dropoff[0]), float(dropoff[1]))
        self.requested_robot = requested_robot if requested_robot else None
        self.reassignment_count = 0
        if isinstance(priority, TaskPriority):
            self.priority = priority
        else:
            self.priority = TaskPriority.from_str(str(priority))
        self.created_at = created_at if created_at is not None else time.time()
        self.release_time_sec = float(release_time_sec)
        self.deadline = float(deadline) if deadline is not None else None
        self.metadata = metadata if metadata is not None else {}

        if self.release_time_sec > 0.0:
            self._state: TaskLifecycleState = TaskLifecycleState.STAGED
            init_event = 'STAGED'
            init_state = TaskLifecycleState.STAGED
            init_details = f'Task staged until release at t={self.release_time_sec:.1f}s'
        else:
            self._state = TaskLifecycleState.PENDING
            init_event = 'CREATED'
            init_state = TaskLifecycleState.PENDING
            init_details = 'Task initialized'

        self.assigned_robot_id: Optional[str] = None
        self.assigned_at: Optional[float] = None
        self.started_at: Optional[float] = None
        self.completed_at: Optional[float] = None
        self.failed_at: Optional[float] = None
        self.cancelled_at: Optional[float] = None

        self._events: List[TaskEvent] = [
            TaskEvent(
                timestamp=self.created_at,
                event_type=init_event,
                from_state=None,
                to_state=init_state,
                details=init_details,
            )
        ]

    @property
    def state(self) -> TaskLifecycleState:
        """Get current lifecycle state."""
        return self._state

    @property
    def is_terminal(self) -> bool:
        """Return True if task is in a terminal state."""
        return self._state in {
            TaskLifecycleState.COMPLETED,
            TaskLifecycleState.FAILED,
            TaskLifecycleState.CANCELLED,
        }

    @property
    def events(self) -> List[TaskEvent]:
        """Return audit history of all lifecycle events."""
        return list(self._events)

    def transition_to(
        self,
        new_state: TaskLifecycleState,
        timestamp: Optional[float] = None,
        robot_id: Optional[str] = None,
        details: str = '',
    ) -> None:
        """
        Transition task to a new state if permissible under the state machine.

        :param new_state: Target TaskLifecycleState.
        :param timestamp: Event timestamp (seconds).
        :param robot_id: Associated robot ID (if assigned or executing).
        :param details: Contextual event details.
        :raises InvalidTaskTransitionError: If transition is forbidden.
        """
        if new_state not in self.VALID_TRANSITIONS[self._state]:
            raise InvalidTaskTransitionError(self._state, new_state, self.task_id)

        now = timestamp if timestamp is not None else time.time()
        old_state = self._state
        self._state = new_state

        if new_state == TaskLifecycleState.ASSIGNED:
            self.assigned_robot_id = robot_id
            self.assigned_at = now
            event_type = 'ASSIGNED'
        elif new_state == TaskLifecycleState.PENDING:
            self.assigned_robot_id = None
            if old_state == TaskLifecycleState.ASSIGNED:
                self.reassignment_count += 1
                event_type = (
                    'REQUEUED'
                    if ('requeue' in details.lower() or 'operator' in details.lower())
                    else 'UNASSIGNED'
                )
            elif old_state == TaskLifecycleState.STAGED:
                event_type = 'RELEASED'
            elif old_state in (TaskLifecycleState.FAILED, TaskLifecycleState.IN_PROGRESS):
                self.reassignment_count += 1
                event_type = 'REQUEUED'
            else:
                event_type = 'REQUEUED'
        elif new_state == TaskLifecycleState.IN_PROGRESS:
            self.started_at = now
            if robot_id:
                self.assigned_robot_id = robot_id
            event_type = 'STARTED'
        elif new_state == TaskLifecycleState.COMPLETED:
            self.completed_at = now
            event_type = 'COMPLETED'
        elif new_state == TaskLifecycleState.FAILED:
            self.failed_at = now
            event_type = 'FAILED'
        elif new_state == TaskLifecycleState.CANCELLED:
            self.cancelled_at = now
            event_type = 'CANCELLED'
        else:
            event_type = 'TRANSITION'

        event = TaskEvent(
            timestamp=now,
            event_type=event_type,
            from_state=old_state,
            to_state=new_state,
            robot_id=robot_id or self.assigned_robot_id,
            details=details,
        )
        self._events.append(event)

    # -----------------------------------------------------------------
    # Metrics & Durations (Truthful, non-fabricated)
    # -----------------------------------------------------------------
    @property
    def waiting_time(self) -> Optional[float]:
        """Duration between task creation and execution start (or assignment)."""
        if self.started_at is not None:
            return round(self.started_at - self.created_at, 3)
        if self.assigned_at is not None:
            return round(self.assigned_at - self.created_at, 3)
        return None

    @property
    def execution_time(self) -> Optional[float]:
        """Duration between execution start and terminal resolution."""
        if self.started_at is None:
            return None
        terminal_time = self.completed_at or self.failed_at or self.cancelled_at
        if terminal_time is not None:
            return round(terminal_time - self.started_at, 3)
        return None

    @property
    def total_duration(self) -> Optional[float]:
        """Total lifecycle span from creation to terminal state."""
        terminal_time = self.completed_at or self.failed_at or self.cancelled_at
        if terminal_time is not None:
            return round(terminal_time - self.created_at, 3)
        return None

    @property
    def deadline_slack(self) -> Optional[float]:
        """Remaining time before deadline at completion (positive=on-time, negative=late)."""
        if self.deadline is None or self.completed_at is None:
            return None
        return round(self.deadline - self.completed_at, 3)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize task entity to dictionary representation."""
        return {
            'task_id': self.task_id,
            'pickup': list(self.pickup),
            'dropoff': list(self.dropoff),
            'priority': self.priority.name,
            'priority_val': int(self.priority),
            'status': self._state.value,
            'created_at': self.created_at,
            'release_time_sec': self.release_time_sec,
            'deadline': self.deadline,
            'requested_robot': self.requested_robot,
            'reassignment_count': self.reassignment_count,
            'assigned_robot_id': self.assigned_robot_id,
            'assigned_at': self.assigned_at,
            'started_at': self.started_at,
            'completed_at': self.completed_at,
            'failed_at': self.failed_at,
            'cancelled_at': self.cancelled_at,
            'waiting_time': self.waiting_time,
            'execution_time': self.execution_time,
            'total_duration': self.total_duration,
            'deadline_slack': self.deadline_slack,
            'metadata': self.metadata,
            'events_count': len(self._events),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Task':
        """Construct Task instance from dictionary."""
        task = cls(
            task_id=data['task_id'],
            pickup=tuple(data['pickup']),
            dropoff=tuple(data['dropoff']),
            priority=TaskPriority.from_str(str(data.get('priority', 'NORMAL'))),
            created_at=data.get('created_at'),
            deadline=data.get('deadline'),
            metadata=data.get('metadata', {}),
            release_time_sec=float(data.get('release_time_sec', 0.0)),
            requested_robot=data.get('requested_robot'),
        )
        task.reassignment_count = int(data.get('reassignment_count', 0))
        status_str = data.get('status')
        if status_str and status_str != TaskLifecycleState.PENDING.value:
            # Replay state transition if non-pending
            target_state = TaskLifecycleState.from_str(status_str)
            if target_state != TaskLifecycleState.PENDING:
                task._state = target_state
                task.assigned_robot_id = data.get('assigned_robot_id')
                task.assigned_at = data.get('assigned_at')
                task.started_at = data.get('started_at')
                task.completed_at = data.get('completed_at')
                task.failed_at = data.get('failed_at')
                task.cancelled_at = data.get('cancelled_at')
        return task


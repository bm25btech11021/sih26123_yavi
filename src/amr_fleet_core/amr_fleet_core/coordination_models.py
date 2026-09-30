"""Data models and representations for multi-agent path coordination (M6)."""

from dataclasses import dataclass, field
from enum import Enum
import time
from typing import List, Tuple

Position = Tuple[int, int]


class ConflictType(str, Enum):
    """Enumeration of multi-agent space-time conflict categories."""

    VERTEX = 'VERTEX'
    EDGE_SWAP = 'EDGE_SWAP'
    WAITING = 'WAITING'
    RESERVATION = 'RESERVATION'


class CoordinationState(str, Enum):
    """Local coordination status for an individual AMR."""

    CLEAR = 'CLEAR'
    CONFLICT_RESOLVED = 'CONFLICT_RESOLVED'
    YIELDING = 'YIELDING'
    BACKTRACKING = 'BACKTRACKING'
    RECOVERING = 'RECOVERING'


@dataclass(frozen=True)
class Reservation:
    """Discrete space-time reservation on vertex or directed edge."""

    robot_id: str
    from_pos: Position
    to_pos: Position
    time_step: int
    duration_sec: float = 1.0
    is_edge: bool = False
    priority: float = 0.0
    created_at: float = field(default_factory=time.time)


@dataclass
class Conflict:
    """Identified space-time conflict between two agents or reservation attempt."""

    conflict_type: str
    robot_a: str
    robot_b: str
    cell: Position
    time_step: int
    details: str = ''
    resolved: bool = False


@dataclass
class DeadlockRecord:
    """Record of a detected multi-agent deadlock event and recovery attempt."""

    cycle_robot_ids: List[str]
    root_cause: str
    persistence_duration_sec: float
    recovery_action: str
    recovery_success: bool = False
    recovery_duration_sec: float = 0.0
    created_at: float = field(default_factory=time.time)


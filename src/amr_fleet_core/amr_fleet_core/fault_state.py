"""Fault states and diagnostic data models for YAVI-SIH26123 (v2)."""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Tuple


class FaultState(Enum):
    """Discrete operational health and fault states for an AMR."""

    HEALTHY = 'HEALTHY'
    DEGRADED = 'DEGRADED'
    COMM_LOSS = 'COMM_LOSS'
    MOTION_FAILURE = 'MOTION_FAILURE'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    FAILED = 'FAILED'
    RECOVERING = 'RECOVERING'

    @classmethod
    def from_str(cls, val: str) -> 'FaultState':
        """Convert string to FaultState."""
        upper = val.strip().upper()
        if upper in cls.__members__:
            return cls[upper]
        return cls.HEALTHY

    @property
    def can_move(self) -> bool:
        """Return True if robot is physically capable of wheel motion."""
        return self in (FaultState.HEALTHY, FaultState.DEGRADED, FaultState.COMM_LOSS)

    @property
    def publishes_heartbeat(self) -> bool:
        """Return True if robot's compute and network stack emit heartbeats."""
        return self != FaultState.FAILED

    @property
    def can_own_tasks(self) -> bool:
        """Return True if robot can retain ownership of assigned tasks."""
        return self in (
            FaultState.HEALTHY,
            FaultState.DEGRADED,
            FaultState.COMM_LOSS,
            FaultState.EMERGENCY_STOP,
        )

    @property
    def eligible_for_cbba(self) -> bool:
        """Return True if robot is actively eligible to bid on new tasks in CBBA."""
        return self in (FaultState.HEALTHY, FaultState.DEGRADED)

    @property
    def holds_reservations(self) -> bool:
        """Return True if robot can maintain space-time corridor reservations."""
        return self in (
            FaultState.HEALTHY,
            FaultState.DEGRADED,
            FaultState.COMM_LOSS,
            FaultState.EMERGENCY_STOP,
        )

    @property
    def is_chassis_obstacle(self) -> bool:
        """Return True if disabled chassis must be rasterized as static obstacle."""
        return self in (FaultState.MOTION_FAILURE, FaultState.FAILED)

    @property
    def is_local_autonomy_permitted(self) -> bool:
        """Return True if robot may operate under local autonomy during disconnection."""
        return self == FaultState.COMM_LOSS


@dataclass
class RobotHealthInfo:
    """Snapshot of a robot's operational health state."""

    robot_id: str
    state: FaultState
    last_pose: Tuple[float, float]
    active_task_id: str
    last_update_time: float
    confirmed_failure: bool = False
    details: str = ''

    def to_dict(self) -> Dict[str, Any]:
        """Serialize health snapshot to dictionary."""
        return {
            'robot_id': self.robot_id,
            'state': self.state.value,
            'last_pose': list(self.last_pose),
            'active_task_id': self.active_task_id,
            'last_update_time': self.last_update_time,
            'confirmed_failure': self.confirmed_failure,
            'details': self.details,
        }


"""Robot lifecycle state machine and transition logic for AMR fleet coordination."""

from enum import auto, Enum
from typing import Dict, List, Set, Tuple


class RobotLifecycleState(Enum):
    """Lifecycle states for individual autonomous mobile robots."""

    UNCONFIGURED = auto()
    IDLE = auto()
    BIDDING = auto()
    PLANNING = auto()
    EXECUTING = auto()
    WAITING = auto()
    RECOVERING = auto()
    FAULT = auto()


class InvalidStateTransitionError(Exception):
    """Raised when an illegal lifecycle state transition is attempted."""

    def __init__(self, from_state: RobotLifecycleState, to_state: RobotLifecycleState):
        super().__init__(
            f'Invalid transition from {from_state.name} to {to_state.name}'
        )
        self.from_state = from_state
        self.to_state = to_state


class RobotStateMachine:
    """Manages verified state transitions for an AMR."""

    VALID_TRANSITIONS: Dict[RobotLifecycleState, Set[RobotLifecycleState]] = {
        RobotLifecycleState.UNCONFIGURED: {
            RobotLifecycleState.IDLE,
            RobotLifecycleState.FAULT,
        },
        RobotLifecycleState.IDLE: {
            RobotLifecycleState.BIDDING,
            RobotLifecycleState.PLANNING,
            RobotLifecycleState.FAULT,
        },
        RobotLifecycleState.BIDDING: {
            RobotLifecycleState.IDLE,
            RobotLifecycleState.PLANNING,
            RobotLifecycleState.FAULT,
        },
        RobotLifecycleState.PLANNING: {
            RobotLifecycleState.EXECUTING,
            RobotLifecycleState.WAITING,
            RobotLifecycleState.IDLE,
            RobotLifecycleState.FAULT,
        },
        RobotLifecycleState.EXECUTING: {
            RobotLifecycleState.IDLE,
            RobotLifecycleState.WAITING,
            RobotLifecycleState.PLANNING,
            RobotLifecycleState.RECOVERING,
            RobotLifecycleState.FAULT,
        },
        RobotLifecycleState.WAITING: {
            RobotLifecycleState.EXECUTING,
            RobotLifecycleState.PLANNING,
            RobotLifecycleState.RECOVERING,
            RobotLifecycleState.IDLE,
            RobotLifecycleState.FAULT,
        },
        RobotLifecycleState.RECOVERING: {
            RobotLifecycleState.PLANNING,
            RobotLifecycleState.IDLE,
            RobotLifecycleState.FAULT,
        },
        RobotLifecycleState.FAULT: {
            RobotLifecycleState.UNCONFIGURED,
            RobotLifecycleState.IDLE,
        },
    }

    def __init__(
        self,
        robot_id: str,
        initial_state: RobotLifecycleState = RobotLifecycleState.UNCONFIGURED,
    ):
        self.robot_id = robot_id
        self._state = initial_state
        self._history: List[Tuple[RobotLifecycleState, RobotLifecycleState]] = []

    @property
    def current_state(self) -> RobotLifecycleState:
        """Get the current robot state."""
        return self._state

    def transition_to(self, new_state: RobotLifecycleState) -> None:
        """
        Transition to a new state if valid.

        :raises InvalidStateTransitionError: If the transition is prohibited.
        """
        allowed = self.VALID_TRANSITIONS.get(self._state, set())
        if new_state not in allowed:
            raise InvalidStateTransitionError(self._state, new_state)

        old_state = self._state
        self._state = new_state
        self._history.append((old_state, new_state))

    @property
    def history(self) -> List[Tuple[RobotLifecycleState, RobotLifecycleState]]:
        """Return transition history as a list of (from_state, to_state) tuples."""
        return list(self._history)


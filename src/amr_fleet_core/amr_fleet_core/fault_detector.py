"""Decentralized Peer Health and Fault Detector for YAVI-SIH26123 (v2)."""

from dataclasses import dataclass
import time
from typing import Dict, List, Optional, Set, Tuple

from amr_fleet_core.fault_state import FaultState, RobotHealthInfo


@dataclass
class FaultDetectorConfig:
    """Configuration thresholds for decentralized failure detection."""

    heartbeat_period_s: float = 0.5
    comm_loss_threshold_s: float = 1.5
    failure_timeout_s: float = 3.5
    confirmation_samples: int = 2


class FaultDetector:
    """
    Decentralized fault detection and peer liveness tracking engine.

    Maintains local self-health state and tracks peer heartbeats without
    relying on any centralized server. Detects peer communication loss,
    hard failure/crash, and tracks last known poses and tasks of failed peers.
    """

    def __init__(
        self,
        local_robot_id: str,
        config: Optional[FaultDetectorConfig] = None,
    ) -> None:
        """Initialize fault detector."""
        self.local_robot_id = local_robot_id
        self.config = config or FaultDetectorConfig()

        # Self health state
        self.self_state: FaultState = FaultState.HEALTHY
        self.injected_fault: Optional[str] = None
        self.fault_expiry_time: Optional[float] = None
        self.self_pose: Tuple[float, float] = (0.0, 0.0)
        self.self_active_task_id: str = ''

        # Peer health state
        self.peer_records: Dict[str, RobotHealthInfo] = {}
        self.peer_fail_confirmations: Dict[str, int] = {}
        self.failed_peers: Set[str] = set()
        self.newly_recovered_peers: Set[str] = set()

    def inject_fault(
        self,
        fault_type: str,
        duration_sec: float = -1.0,
        now: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """
        Inject a simulated fault condition for testing.

        :param fault_type: "KILL", "MOTION_STOP", "COMM_ISOLATE", "CLEAR"
        :param duration_sec: Duration in seconds (-1.0 for indefinite)
        :param now: Current epoch timestamp
        :return: Tuple of (success, message)
        """
        current_time = now if now is not None else time.time()
        ft_upper = fault_type.strip().upper()

        if ft_upper in (
            'CLEAR', 'RESET', 'NONE', 'RESTORE', 'RECOVER', 'RECONNECT', 'ONLINE'
        ):
            self.clear_fault()
            return True, f'[{self.local_robot_id}] Fault cleared. Restored to HEALTHY.'

        if ft_upper in ('KILL', 'FAILED', 'CRASH'):
            self.self_state = FaultState.FAILED
            self.injected_fault = 'KILL'
        elif ft_upper in ('MOTION_STOP', 'MOTION_FAILURE', 'MOTOR_FAIL'):
            self.self_state = FaultState.MOTION_FAILURE
            self.injected_fault = 'MOTION_STOP'
        elif ft_upper in (
            'COMM_ISOLATE', 'COMM_LOSS', 'NETWORK_CUT', 'DISCONNECT',
            'OUTAGE', 'NETWORK_OUTAGE', 'NETWORK_LOSS'
        ):
            self.self_state = FaultState.COMM_LOSS
            self.injected_fault = 'COMM_ISOLATE'
        elif ft_upper in ('EMERGENCY_STOP', 'E_STOP', 'ESTOP'):
            self.self_state = FaultState.EMERGENCY_STOP
            self.injected_fault = 'EMERGENCY_STOP'
        elif ft_upper == 'RECOVERING':
            self.self_state = FaultState.RECOVERING
            self.injected_fault = 'RECOVERING'
        else:
            return False, f"Unknown fault type '{fault_type}'"

        if duration_sec > 0.0:
            self.fault_expiry_time = current_time + duration_sec
        else:
            self.fault_expiry_time = None

        msg = (
            f'[{self.local_robot_id}] Fault {self.injected_fault} injected '
            f'(state={self.self_state.value}).'
        )
        return True, msg

    def clear_fault(self) -> None:
        """Clear active injected fault and restore self to HEALTHY."""
        self.self_state = FaultState.HEALTHY
        self.injected_fault = None
        self.fault_expiry_time = None

    def update_self_telemetry(
        self,
        pose: Tuple[float, float],
        active_task_id: str = '',
        now: Optional[float] = None,
    ) -> None:
        """Update local robot telemetry."""
        self.self_pose = (round(pose[0], 3), round(pose[1], 3))
        self.self_active_task_id = active_task_id

        current_time = now if now is not None else time.time()
        if self.fault_expiry_time is not None and current_time >= self.fault_expiry_time:
            self.clear_fault()

    def record_peer_heartbeat(
        self,
        robot_id: str,
        state_str: str,
        pose: Tuple[float, float],
        active_task_id: str = '',
        timestamp: Optional[float] = None,
    ) -> None:
        """Record received heartbeat from a peer AMR."""
        if robot_id == self.local_robot_id:
            return

        now = timestamp if timestamp is not None else time.time()
        peer_state = FaultState.from_str(state_str)

        record = self.peer_records.get(robot_id)
        if record is None:
            record = RobotHealthInfo(
                robot_id=robot_id,
                state=peer_state,
                last_pose=(round(pose[0], 3), round(pose[1], 3)),
                active_task_id=active_task_id,
                last_update_time=now,
                confirmed_failure=(peer_state == FaultState.FAILED),
            )
            self.peer_records[robot_id] = record
        else:
            record.state = peer_state
            record.last_pose = (round(pose[0], 3), round(pose[1], 3))
            if active_task_id:
                record.active_task_id = active_task_id
            record.last_update_time = now
            if peer_state != FaultState.FAILED and record.confirmed_failure:
                record.confirmed_failure = False
                self.failed_peers.discard(robot_id)
                self.peer_fail_confirmations[robot_id] = 0
                self.newly_recovered_peers.add(robot_id)

        # If peer explicitly reports itself as FAILED:
        if peer_state == FaultState.FAILED:
            record.confirmed_failure = True
            self.failed_peers.add(robot_id)

    def evaluate_peers(
        self,
        now: Optional[float] = None,
    ) -> Tuple[List[str], List[str]]:
        """
        Evaluate freshness of peer heartbeats.

        :return: Tuple of (newly_failed_peer_ids, newly_recovered_peer_ids).
        """
        current_time = now if now is not None else time.time()
        newly_failed: List[str] = []
        newly_recovered: List[str] = list(self.newly_recovered_peers)
        self.newly_recovered_peers.clear()

        for p_id, rec in self.peer_records.items():
            dt = max(0.0, current_time - rec.last_update_time)

            if rec.confirmed_failure:
                # Already confirmed failed
                continue

            if dt >= self.config.failure_timeout_s:
                # Increment debounce confirmation
                count = self.peer_fail_confirmations.get(p_id, 0) + 1
                self.peer_fail_confirmations[p_id] = count

                if count >= self.config.confirmation_samples:
                    rec.confirmed_failure = True
                    rec.state = FaultState.FAILED
                    self.failed_peers.add(p_id)
                    newly_failed.append(p_id)
            elif dt >= self.config.comm_loss_threshold_s:
                rec.state = FaultState.COMM_LOSS
                self.peer_fail_confirmations[p_id] = 0
            else:
                if rec.state == FaultState.COMM_LOSS:
                    rec.state = FaultState.HEALTHY
                self.peer_fail_confirmations[p_id] = 0

        return newly_failed, newly_recovered

    def is_peer_failed(self, robot_id: str) -> bool:
        """Return True if peer is confirmed failed."""
        return robot_id in self.failed_peers

    def get_failed_peers(self) -> Set[str]:
        """Return set of all confirmed failed peer IDs."""
        return set(self.failed_peers)

    def is_self_healthy(self) -> bool:
        """Return True if local robot is HEALTHY and can execute tasks."""
        return self.self_state == FaultState.HEALTHY

    def get_peer_pose(self, robot_id: str) -> Optional[Tuple[float, float]]:
        """Get last known coordinates of peer robot."""
        rec = self.peer_records.get(robot_id)
        return rec.last_pose if rec else None

    def get_peer_task(self, robot_id: str) -> Optional[str]:
        """Get last known active task ID held by peer robot."""
        rec = self.peer_records.get(robot_id)
        return rec.active_task_id if rec and rec.active_task_id else None

    def is_in_comm_loss(self) -> bool:
        """Return True if local robot is experiencing communication loss."""
        return self.self_state == FaultState.COMM_LOSS

    def is_local_autonomy_active(self) -> bool:
        """Return True if local robot is operating under local autonomy."""
        return self.self_state == FaultState.COMM_LOSS and self.self_state.can_move

    def is_peer_in_comm_loss(self, robot_id: str) -> bool:
        """Return True if peer robot is currently in COMM_LOSS."""
        rec = self.peer_records.get(robot_id)
        return rec is not None and rec.state == FaultState.COMM_LOSS


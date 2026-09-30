"""Deterministic Adaptive Compute Policy for Decentralized AMR Fleet Coordination."""

from dataclasses import dataclass
import time
from typing import Dict, List, Optional, Tuple

from amr_fleet_core.compute_modes import ComputeMode


@dataclass
class ComputeState:
    """Measurable system telemetry fed to the adaptive compute policy."""

    timestamp: float
    information_age_s: float = 0.0          # Max age of received peer reservations (s)
    comm_latency_ms: float = 0.0            # Average or P95 communication latency (ms)
    comm_loss_rate: float = 0.0             # Packet drop ratio in [0.0, 1.0]
    active_conflicts: int = 0               # Recent space-time conflicts detected
    is_yielding_or_recovering: bool = False  # Flag for deadlock yield / lateral escape
    planning_latency_ms: float = 0.0        # Execution time of last A* planning cycle
    cpu_utilization_pct: float = 0.0        # Host or process CPU percentage
    safety_active: bool = False             # LiDAR reactive brake active


@dataclass
class PolicyTransitionRecord:
    """Audit record capturing the full context of a compute mode transition."""

    timestamp: float
    robot_id: str
    previous_mode: ComputeMode
    new_mode: ComputeMode
    trigger_signal: str
    trigger_value: float
    threshold_value: float
    reason: str
    dwell_time_sec: float

    def to_dict(self) -> Dict[str, object]:
        """Serialize record to dictionary for telemetry logging."""
        return {
            'timestamp': round(self.timestamp, 3),
            'robot_id': self.robot_id,
            'previous_mode': self.previous_mode.value,
            'new_mode': self.new_mode.value,
            'trigger_signal': self.trigger_signal,
            'trigger_value': round(self.trigger_value, 3),
            'threshold_value': round(self.threshold_value, 3),
            'reason': self.reason,
            'dwell_time_sec': round(self.dwell_time_sec, 3),
        }


@dataclass
class AdaptiveComputePolicyConfig:
    """Thresholds and hysteresis parameters for adaptive compute mode selection."""

    min_dwell_time_sec: float = 3.0             # Minimum duration in mode before switching
    confirmation_samples: int = 3               # Consecutive cycles required to confirm switch
    stale_age_low_threshold_s: float = 1.5      # Stale age to trigger LOW (M7 operating threshold)
    stale_age_normal_recovery_s: float = 0.8    # Stale age to recover to NORMAL
    loss_rate_low_threshold: float = 0.15       # Packet loss rate to trigger LOW
    loss_rate_normal_recovery: float = 0.05     # Packet loss rate to recover to NORMAL
    conflict_high_threshold: int = 2            # Conflicts in window to trigger HIGH
    cpu_high_threshold_pct: float = 85.0        # CPU utilization to trigger LOW
    cpu_normal_recovery_pct: float = 65.0       # CPU utilization to recover to NORMAL
    planning_latency_low_threshold_ms: float = 15.0  # Planning latency to trigger LOW
    telemetry_timeout_s: float = 5.0            # Telemetry timeout before fail-safe to NORMAL


class AdaptiveComputePolicy:
    """
    Deterministic decision engine modulating computational mode based on real-time signals.

    Features:
    - Independent anti-oscillation (dwell time & consecutive sample confirmation)
    - Asymmetric hysteresis for network degradation and recovery
    - Priority-ordered signal evaluation (Degraded Network > Compute Pressure > Traffic Contention)
    - Fail-safe fallback to COMPUTE_MODE_NORMAL on missing data, errors, or timeouts
    - Complete auditability via structured transition records
    """

    def __init__(
        self,
        robot_id: str,
        config: Optional[AdaptiveComputePolicyConfig] = None,
        initial_mode: ComputeMode = ComputeMode.NORMAL,
    ) -> None:
        self.robot_id = robot_id
        self.config = config or AdaptiveComputePolicyConfig()
        self.current_mode: ComputeMode = initial_mode
        self.last_transition_time: float = 0.0
        self.last_state_time: float = 0.0

        # Anti-oscillation state
        self.candidate_mode: Optional[ComputeMode] = None
        self.candidate_confirmations: int = 0
        self.candidate_reason: str = ''
        self.candidate_signal: str = ''
        self.candidate_trigger_val: float = 0.0
        self.candidate_threshold_val: float = 0.0

        # Audit and telemetry history
        self.transition_history: List[PolicyTransitionRecord] = []
        self.mode_durations: Dict[ComputeMode, float] = {
            ComputeMode.LOW: 0.0,
            ComputeMode.NORMAL: 0.0,
            ComputeMode.HIGH: 0.0,
        }
        self.last_eval_time: float = 0.0

    def evaluate(
        self,
        state: Optional[ComputeState],
    ) -> Tuple[ComputeMode, Optional[PolicyTransitionRecord]]:
        """
        Evaluate system state and return (active_mode, transition_record_if_switched).

        Guaranteed deterministic for identical input sequences.
        Guaranteed fail-safe to COMPUTE_MODE_NORMAL if state is invalid or stale.
        """
        now = time.time() if state is None or state.timestamp <= 0.0 else state.timestamp

        # Track mode duration
        if self.last_eval_time > 0.0:
            dt = max(0.0, now - self.last_eval_time)
            self.mode_durations[self.current_mode] = (
                self.mode_durations.get(self.current_mode, 0.0) + dt
            )
        self.last_eval_time = now

        # Fail-Safe 1: Null or invalid state -> Fallback to NORMAL
        if state is None:
            return self._fail_safe_fallback(now, 'NULL_STATE')

        # Fail-Safe 2: Stale telemetry -> Fallback to NORMAL
        if self.last_state_time > 0.0:
            if (now - self.last_state_time) > self.config.telemetry_timeout_s:
                self.last_state_time = now
                return self._fail_safe_fallback(now, 'TELEMETRY_TIMEOUT')
        self.last_state_time = now

        # Check dwell time constraint
        time_in_mode = (
            now - self.last_transition_time
            if self.last_transition_time > 0.0 else float('inf')
        )
        is_dwell_active = time_in_mode < self.config.min_dwell_time_sec

        # Determine target mode candidate based on priority rules
        target_mode, signal, val, thresh, reason = self._determine_candidate_mode(state)

        # If target matches current mode, reset candidate tracking
        if target_mode == self.current_mode:
            self.candidate_mode = None
            self.candidate_confirmations = 0
            return self.current_mode, None

        # If dwell time is active, candidate cannot trigger transition yet
        if is_dwell_active:
            return self.current_mode, None

        # Anti-oscillation confirmation filter
        if target_mode == self.candidate_mode:
            self.candidate_confirmations += 1
        else:
            self.candidate_mode = target_mode
            self.candidate_confirmations = 1
            self.candidate_signal = signal
            self.candidate_trigger_val = val
            self.candidate_threshold_val = thresh
            self.candidate_reason = reason

        if self.candidate_confirmations >= self.config.confirmation_samples:
            # Transition confirmed!
            prev_mode = self.current_mode
            self.current_mode = target_mode
            self.last_transition_time = now
            record = PolicyTransitionRecord(
                timestamp=now,
                robot_id=self.robot_id,
                previous_mode=prev_mode,
                new_mode=target_mode,
                trigger_signal=self.candidate_signal,
                trigger_value=self.candidate_trigger_val,
                threshold_value=self.candidate_threshold_val,
                reason=self.candidate_reason,
                dwell_time_sec=time_in_mode if time_in_mode != float('inf') else 0.0,
            )
            self.transition_history.append(record)
            self.candidate_mode = None
            self.candidate_confirmations = 0
            return self.current_mode, record

        return self.current_mode, None

    def _determine_candidate_mode(
        self,
        state: ComputeState,
    ) -> Tuple[ComputeMode, str, float, float, str]:
        """
        Evaluate prioritized rules to find candidate target mode.

        Priority 1: Network / Stale-State Degradation -> COMPUTE_MODE_LOW
        Priority 2: Compute / CPU Saturation -> COMPUTE_MODE_LOW
        Priority 3: Spatial Traffic Contention -> COMPUTE_MODE_HIGH
        Priority 4: Nominal Recovery -> COMPUTE_MODE_NORMAL
        """
        cfg = self.config

        # Rule 1: Degraded communication or stale distributed state
        if state.information_age_s >= cfg.stale_age_low_threshold_s:
            return (
                ComputeMode.LOW,
                'information_age_s',
                state.information_age_s,
                cfg.stale_age_low_threshold_s,
                'STALE_DISTRIBUTED_RESERVATIONS_DETECTED',
            )
        if state.comm_loss_rate >= cfg.loss_rate_low_threshold:
            return (
                ComputeMode.LOW,
                'comm_loss_rate',
                state.comm_loss_rate,
                cfg.loss_rate_low_threshold,
                'HIGH_PACKET_LOSS_RATE',
            )

        # Rule 2: Compute pressure (Host CPU or A* Planning Latency)
        if state.cpu_utilization_pct >= cfg.cpu_high_threshold_pct:
            return (
                ComputeMode.LOW,
                'cpu_utilization_pct',
                state.cpu_utilization_pct,
                cfg.cpu_high_threshold_pct,
                'HOST_CPU_SATURATION',
            )
        if state.planning_latency_ms >= cfg.planning_latency_low_threshold_ms:
            return (
                ComputeMode.LOW,
                'planning_latency_ms',
                state.planning_latency_ms,
                cfg.planning_latency_low_threshold_ms,
                'PLANNING_CYCLE_LATENCY_OVERRUN',
            )

        # Rule 3: High spatial contention under healthy network
        # (Only enter HIGH if network is clear of stale reservations and packet drops)
        is_network_healthy = (
            state.information_age_s <= cfg.stale_age_normal_recovery_s
            and state.comm_loss_rate <= cfg.loss_rate_normal_recovery
        )
        if is_network_healthy:
            if state.active_conflicts >= cfg.conflict_high_threshold:
                return (
                    ComputeMode.HIGH,
                    'active_conflicts',
                    float(state.active_conflicts),
                    float(cfg.conflict_high_threshold),
                    'SPACE_TIME_CONFLICTS_IN_LOCAL_WINDOW',
                )
            if state.is_yielding_or_recovering:
                return (
                    ComputeMode.HIGH,
                    'is_yielding_or_recovering',
                    1.0,
                    1.0,
                    'DEADLOCK_YIELD_OR_ESCAPE_ACTIVE',
                )

        # Rule 4: Nominal state / recovery
        if self.current_mode == ComputeMode.LOW:
            # Recover to NORMAL only when both network and CPU are sustainably clear
            is_comm_recovered = (
                state.information_age_s <= cfg.stale_age_normal_recovery_s
                and state.comm_loss_rate <= cfg.loss_rate_normal_recovery
            )
            is_cpu_recovered = state.cpu_utilization_pct <= cfg.cpu_normal_recovery_pct
            if is_comm_recovered and is_cpu_recovered:
                return (
                    ComputeMode.NORMAL,
                    'comm_and_cpu_recovered',
                    state.information_age_s,
                    cfg.stale_age_normal_recovery_s,
                    'COMMUNICATION_AND_CPU_RECOVERED_TO_NOMINAL',
                )
            # Otherwise stay in LOW
            return ComputeMode.LOW, 'maintaining_low', 0.0, 0.0, 'DEGRADATION_PERSISTS'

        if self.current_mode == ComputeMode.HIGH:
            # Recover to NORMAL only when conflicts clear
            if state.active_conflicts == 0 and not state.is_yielding_or_recovering:
                return (
                    ComputeMode.NORMAL,
                    'active_conflicts',
                    0.0,
                    0.0,
                    'SPATIAL_CONTENTION_RESOLVED',
                )
            return ComputeMode.HIGH, 'maintaining_high', 0.0, 0.0, 'CONTENTION_PERSISTS'

        # Already in NORMAL and nominal conditions prevail
        return ComputeMode.NORMAL, 'nominal', 0.0, 0.0, 'NOMINAL_CONDITIONS'

    def _fail_safe_fallback(
        self,
        now: float,
        reason: str,
    ) -> Tuple[ComputeMode, Optional[PolicyTransitionRecord]]:
        """Force immediate fallback to COMPUTE_MODE_NORMAL upon exception or fault."""
        if self.current_mode != ComputeMode.NORMAL:
            prev = self.current_mode
            self.current_mode = ComputeMode.NORMAL
            self.last_transition_time = now
            record = PolicyTransitionRecord(
                timestamp=now,
                robot_id=self.robot_id,
                previous_mode=prev,
                new_mode=ComputeMode.NORMAL,
                trigger_signal='fail_safe',
                trigger_value=0.0,
                threshold_value=0.0,
                reason=f'FAIL_SAFE_FALLBACK_{reason}',
                dwell_time_sec=0.0,
            )
            self.transition_history.append(record)
            self.candidate_mode = None
            self.candidate_confirmations = 0
            return self.current_mode, record
        return self.current_mode, None

    def get_occupancy_stats(self, total_duration: float) -> Dict[str, float]:
        """Compute percentage occupancy of each compute mode over the trial."""
        if total_duration <= 0.0:
            total_duration = sum(self.mode_durations.values())
        if total_duration <= 0.0:
            return {
                'mode_occupancy_low_pct': 0.0,
                'mode_occupancy_normal_pct': 100.0,
                'mode_occupancy_high_pct': 0.0,
            }
        return {
            'mode_occupancy_low_pct': round(
                (self.mode_durations.get(ComputeMode.LOW, 0.0) / total_duration) * 100.0, 1
            ),
            'mode_occupancy_normal_pct': round(
                (self.mode_durations.get(ComputeMode.NORMAL, 0.0) / total_duration) * 100.0, 1
            ),
            'mode_occupancy_high_pct': round(
                (self.mode_durations.get(ComputeMode.HIGH, 0.0) / total_duration) * 100.0, 1
            ),
        }

    def reset(self) -> None:
        """Reset policy state for repeated benchmark trial isolation."""
        self.current_mode = ComputeMode.NORMAL
        self.last_transition_time = 0.0
        self.last_state_time = 0.0
        self.candidate_mode = None
        self.candidate_confirmations = 0
        self.transition_history.clear()
        self.mode_durations = {
            ComputeMode.LOW: 0.0,
            ComputeMode.NORMAL: 0.0,
            ComputeMode.HIGH: 0.0,
        }
        self.last_eval_time = 0.0


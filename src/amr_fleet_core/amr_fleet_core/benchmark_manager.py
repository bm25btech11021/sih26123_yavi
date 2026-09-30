"""M8A Benchmark Manager: Reproducible experimentation and aggregation."""

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from enum import Enum
import math
import subprocess
from typing import Any, Dict, List, Optional, Tuple


class TerminationReason(Enum):
    """Discrete mission termination reasons for experimental trials."""

    ALL_TASKS_COMPLETED = 'ALL_TASKS_COMPLETED'
    HORIZON_REACHED = 'HORIZON_REACHED'
    EXPERIMENT_ERROR = 'EXPERIMENT_ERROR'
    SAFETY_ABORT = 'SAFETY_ABORT'

    @classmethod
    def from_str(cls, val: str) -> 'TerminationReason':
        """Convert string to TerminationReason."""
        upper = val.strip().upper()
        if upper in cls.__members__:
            return cls[upper]
        return cls.EXPERIMENT_ERROR


# Student's t critical values for 95% confidence intervals (df = n - 1)
T_CRIT_95: Dict[int, float] = {
    1: 12.706,
    2: 4.303,
    3: 3.182,
    4: 2.776,
    5: 2.571,
    6: 2.447,
    7: 2.365,
    8: 2.306,
    9: 2.262,
    10: 2.228,
    11: 2.201,
    12: 2.179,
    13: 2.160,
    14: 2.145,
    15: 2.131,
    16: 2.120,
    17: 2.110,
    18: 2.101,
    19: 2.093,
    20: 2.086,
    25: 2.060,
    30: 2.042,
}


def get_t_crit_95(df: int) -> float:
    """Return Student's t critical value for 95% confidence interval."""
    if df in T_CRIT_95:
        return T_CRIT_95[df]
    if df > 30:
        return 1.960
    for k in sorted(T_CRIT_95.keys()):
        if df <= k:
            return T_CRIT_95[k]
    return 1.960


def get_git_commit() -> str:
    """Retrieve current short git commit hash, or 'unknown'."""
    try:
        out = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            stderr=subprocess.DEVNULL,
            timeout=2.0,
        )
        return out.decode('utf-8').strip()
    except Exception:
        return 'unknown'


def check_obb_intersection(
    p1: Tuple[float, float],
    yaw1: float,
    p2: Tuple[float, float],
    yaw2: float,
    length: float = 0.65,
    width: float = 0.45,
) -> bool:
    """Check if two 2D oriented bounding boxes intersect using Separating Axis Theorem."""
    hl = length / 2.0
    hw = width / 2.0
    local_corners = [
        (hl, hw),
        (-hl, hw),
        (-hl, -hw),
        (hl, -hw),
    ]

    def get_world_corners(center: Tuple[float, float], yaw: float) -> List[Tuple[float, float]]:
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        return [
            (
                center[0] + dx * cos_y - dy * sin_y,
                center[1] + dx * sin_y + dy * cos_y,
            )
            for dx, dy in local_corners
        ]

    corners1 = get_world_corners(p1, yaw1)
    corners2 = get_world_corners(p2, yaw2)

    axes = [
        (math.cos(yaw1), math.sin(yaw1)),
        (-math.sin(yaw1), math.cos(yaw1)),
        (math.cos(yaw2), math.sin(yaw2)),
        (-math.sin(yaw2), math.cos(yaw2)),
    ]

    for ax, ay in axes:
        p1_proj = [c[0] * ax + c[1] * ay for c in corners1]
        p2_proj = [c[0] * ax + c[1] * ay for c in corners2]
        if max(p1_proj) < min(p2_proj) or max(p2_proj) < min(p1_proj):
            return False
    return True


@dataclass
class BenchmarkConfig:
    """Configuration parameters defining a specific experimental trial."""

    workload_size: int = 15
    workload_file: str = 'config/workloads/workload_15_tasks.yaml'
    seed: int = 42
    trial_id: int = 0
    mission_horizon_sec: float = 60.0
    communication_profile: str = 'NORMAL'
    fleet_size: int = 5
    map_id: str = 'warehouse_grid_small'


@dataclass
class ExperimentMetadata:
    """Machine-readable metadata identifying the trial execution context."""

    experiment_id: str
    timestamp_utc: str
    timestamp_unix: float
    git_commit: str
    software_version: str = 'm8a.v1'
    fleet_size: int = 5
    workload_id: str = 'workload_15_tasks'
    workload_size: int = 15
    trial_id: int = 0
    seed: int = 42
    mission_horizon_sec: float = 60.0
    actual_duration_sec: float = 0.0
    communication_profile: str = 'NORMAL'
    map_id: str = 'warehouse_grid_small'
    termination_reason: str = 'HORIZON_REACHED'


@dataclass
class StandardMetrics:
    """Standard empirical metrics measured during a benchmark trial."""

    # Task metrics
    generated_tasks: int = 0
    assigned_tasks: int = 0
    completed_tasks: int = 0
    failed_tasks: int = 0
    remaining_tasks: int = 0
    completion_rate_pct: float = 0.0
    throughput_tasks_per_min: float = 0.0
    makespan_sec: float = 0.0

    # Planning metrics
    planning_cycles: int = 0
    replan_count: int = 0
    planning_latency_mean_ms: Optional[float] = None
    planning_latency_p50_ms: Optional[float] = None
    planning_latency_p95_ms: Optional[float] = None
    planning_latency_p99_ms: Optional[float] = None

    # Allocation metrics
    cbba_convergence_time_ms: Optional[float] = None
    allocation_changes: int = 0
    unassigned_tasks: int = 0

    # Coordination metrics
    conflicts_detected: int = 0
    conflicts_resolved: int = 0
    deadlocks_detected: int = 0
    deadlocks_recovered: int = 0

    # Communication metrics
    packets_sent: int = 0
    packets_delivered: int = 0
    packets_dropped: int = 0
    observed_loss_rate_pct: float = 0.0
    transport_latency_mean_ms: float = 0.0
    transport_latency_p95_ms: float = 0.0
    information_age_mean_s: float = 0.0

    # Safety metrics
    minimum_center_to_center_distance_m: float = 0.0
    minimum_inter_robot_distance_m: float = 0.0
    proximity_breaches: int = 0
    physical_gazebo_contacts: int = 0
    obb_chassis_overlap_samples: int = 0
    collision_contact_events: int = 0
    safety_brake_interventions: int = 0
    safety_aborts: int = 0

    # Compute metrics (psutil)
    cpu_utilization_mean_pct: Optional[float] = None
    cpu_utilization_max_pct: Optional[float] = None
    ram_utilization_mean_mb: Optional[float] = None
    ram_utilization_max_mb: Optional[float] = None

    # M8B Adaptive Compute metrics
    mode_transitions: int = 0
    failsafe_fallbacks: int = 0
    mode_occupancy_low_pct: float = 0.0
    mode_occupancy_normal_pct: float = 100.0
    mode_occupancy_high_pct: float = 0.0


@dataclass
class StatisticalSummary:
    """Statistical summary metrics for an empirical sample."""

    n: int
    mean: float
    median: float
    std: float
    min_val: float
    max_val: float
    p50: float
    p95: Optional[float] = None
    ci_95_lower: Optional[float] = None
    ci_95_upper: Optional[float] = None


class StatisticalAggregator:
    """Computes statistical summaries across repeated benchmark trials."""

    @staticmethod
    def summarize_series(values: List[float]) -> Optional[Dict[str, Any]]:
        """Compute statistical summary for a series of observations."""
        clean = [
            float(v) for v in values
            if v is not None and not math.isnan(v)
        ]
        n = len(clean)
        if n == 0:
            return None

        clean.sort()
        mean_val = round(sum(clean) / n, 4)
        if n % 2 == 1:
            median_val = round(clean[n // 2], 4)
        else:
            mid_sum = clean[n // 2 - 1] + clean[n // 2]
            median_val = round(mid_sum / 2.0, 4)
        min_v = round(clean[0], 4)
        max_v = round(clean[-1], 4)

        if n > 1:
            variance = sum((x - mean_val) ** 2 for x in clean) / (n - 1)
            std_val = round(math.sqrt(variance), 4)
        else:
            std_val = 0.0

        p50_val = median_val

        p95_val = None
        if n >= 5:
            idx = int(math.ceil(0.95 * n)) - 1
            p95_val = round(clean[min(idx, n - 1)], 4)

        ci_lower = None
        ci_upper = None
        if n >= 3 and std_val > 0.0:
            df = n - 1
            t_crit = get_t_crit_95(df)
            sem = std_val / math.sqrt(n)
            margin = t_crit * sem
            ci_lower = round(mean_val - margin, 4)
            ci_upper = round(mean_val + margin, 4)

        return asdict(StatisticalSummary(
            n=n,
            mean=mean_val,
            median=median_val,
            std=std_val,
            min_val=min_v,
            max_val=max_v,
            p50=p50_val,
            p95=p95_val,
            ci_95_lower=ci_lower,
            ci_95_upper=ci_upper,
        ))

    @classmethod
    def aggregate_trials(
        cls,
        benchmark_id: str,
        trials: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Combine multiple raw trial records into aggregated summary."""
        if not trials:
            raise ValueError('No trial records provided for aggregation.')

        first_meta = trials[0].get('metadata', {})
        workload_size = first_meta.get('workload_size', 0)
        comm_profile = first_meta.get('communication_profile', 'NORMAL')
        horizon = first_meta.get('mission_horizon_sec', 0.0)
        compute_mode = first_meta.get('compute_mode', 'NORMAL')
        schema_version = first_meta.get('software_version', 'm8a.v1')

        metric_keys = [
            'completed_tasks',
            'completion_rate_pct',
            'throughput_tasks_per_min',
            'makespan_sec',
            'replan_count',
            'planning_latency_mean_ms',
            'planning_latency_p95_ms',
            'cbba_convergence_time_ms',
            'conflicts_resolved',
            'deadlocks_recovered',
            'minimum_center_to_center_distance_m',
            'minimum_inter_robot_distance_m',
            'proximity_breaches',
            'physical_gazebo_contacts',
            'obb_chassis_overlap_samples',
            'collision_contact_events',
            'safety_brake_interventions',
            'safety_aborts',
            'cpu_utilization_mean_pct',
            'ram_utilization_mean_mb',
            'mode_transitions',
            'failsafe_fallbacks',
            'mode_occupancy_low_pct',
            'mode_occupancy_normal_pct',
            'mode_occupancy_high_pct',
        ]

        series_data: Dict[str, List[float]] = {k: [] for k in metric_keys}
        trial_summaries = []

        for tr in trials:
            m = tr.get('metrics', {})
            meta = tr.get('metadata', {})
            trial_summaries.append({
                'trial_id': meta.get('trial_id'),
                'seed': meta.get('seed'),
                'experiment_id': meta.get('experiment_id'),
                'compute_mode': meta.get('compute_mode', 'NORMAL'),
                'duration_sec': meta.get('actual_duration_sec'),
                'termination_reason': meta.get('termination_reason'),
                'completed_tasks': m.get('completed_tasks'),
                'throughput_tasks_per_min': m.get('throughput_tasks_per_min'),
                'makespan_sec': m.get('makespan_sec'),
                'replan_count': m.get('replan_count'),
                'min_dist_m': m.get('minimum_inter_robot_distance_m'),
                'min_center_to_center_dist_m': m.get(
                    'minimum_center_to_center_distance_m',
                    m.get('minimum_inter_robot_distance_m'),
                ),
                'proximity_breaches': m.get('proximity_breaches', 0),
                'physical_contacts': m.get('physical_gazebo_contacts', 0),
                'obb_chassis_overlap_samples': m.get(
                    'obb_chassis_overlap_samples',
                    m.get('physical_gazebo_contacts', 0),
                ),
                'safety_brakes': m.get('safety_brake_interventions', 0),
                'safety_aborts': m.get(
                    'safety_aborts',
                    1 if meta.get('termination_reason') == 'SAFETY_ABORT' else 0,
                ),
                'collisions': m.get('collision_contact_events'),
                'mode_transitions': m.get('mode_transitions', 0),
                'occupancy_low_pct': m.get('mode_occupancy_low_pct', 0.0),
                'occupancy_normal_pct': m.get(
                    'mode_occupancy_normal_pct', 100.0
                ),
                'occupancy_high_pct': m.get('mode_occupancy_high_pct', 0.0),
            })

            for k in metric_keys:
                v = m.get(k)
                if v is not None:
                    series_data[k].append(v)

        aggregated_metrics = {}
        for k in metric_keys:
            aggregated_metrics[k] = cls.summarize_series(series_data[k])

        return {
            'schema_version': schema_version,
            'benchmark_id': benchmark_id,
            'aggregated_at_utc': datetime.now(timezone.utc).isoformat(),
            'trial_count': len(trials),
            'workload_size': workload_size,
            'communication_profile': comm_profile,
            'compute_mode': compute_mode,
            'configured_horizon_sec': horizon,
            'aggregated_metrics': aggregated_metrics,
            'individual_trials': trial_summaries,
        }


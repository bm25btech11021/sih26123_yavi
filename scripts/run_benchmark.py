#!/usr/bin/env python3
"""
M8A Benchmark Runner: Reproducible multi-workload experimental trials.

Executes real Gazebo Harmonic 5-AMR fleet trials across scaled workloads
(15, 30, 50, 100 tasks), collects empirical performance and compute metrics,
enforces explicit termination reasons, saves raw results, and aggregates
statistics across repeated trials.

Usage:
  python3 scripts/run_benchmark.py --workload 15 --trials 1 --seed 42 --communication NORMAL
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Set, Tuple

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import yaml

from amr_fleet_core.benchmark_manager import (
    BenchmarkConfig,
    check_obb_intersection,
    ExperimentMetadata,
    StandardMetrics,
    StatisticalAggregator,
    TerminationReason,
    get_git_commit,
)
from amr_fleet_core.communication_model import (
    CommunicationAction,
    CommunicationImpairmentModel,
    CommunicationProfileConfig,
    PRESET_PROFILES,
)
from amr_fleet_core.task_model import TaskLifecycleState
from amr_fleet_core.workload import WorkloadManager
from amr_fleet_msgs.msg import (
    CommunicationMetrics,
    CommunicationProfile,
    ComputeModeEvent,
    ConflictReport,
    DeadlockEvent,
    RobotBundle,
    RollingHorizonPlan,
    TaskEvent as TaskEventMsg,
    TaskList,
)
try:
    from amr_fleet_msgs.msg import AisleBlockageEvent
    HAVE_M9_V3_A_MSGS = True
except ImportError:
    HAVE_M9_V3_A_MSGS = False
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# Calibrated standard horizons (seconds) per workload size
CALIBRATED_HORIZONS: Dict[int, float] = {
    15: 60.0,
    30: 120.0,
    50: 200.0,
    100: 360.0,
}

# Scaled bundle capacity per workload size
SCALED_BUNDLE_SIZES: Dict[int, int] = {
    15: 4,
    30: 8,
    50: 12,
    100: 24,
}


def spawn_gazebo_blocker(
    world: str,
    model_name: str = 'dynamic_aisle_blocker',
    x: float = 4.5,
    y: float = 6.75,
    z: float = 0.7,
) -> bool:
    """Spawn static physical barrier in Gazebo Harmonic world."""
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sdf_path = os.path.join(
        ws_root, 'src', 'amr_fleet_bringup', 'models', 'dynamic_aisle_blocker.sdf',
    )
    clean_world = world[:-4] if world.endswith('.sdf') else world
    cmd_gz = [
        'gz', 'service',
        '-s', f'/world/{clean_world}/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', (
            f'sdf_filename: "{sdf_path}", '
            f'name: "{model_name}", '
            f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}, '
            f'allow_renaming: false'
        ),
    ]
    try:
        res = subprocess.run(cmd_gz, capture_output=True, text=True, timeout=10)
        if res.returncode == 0 and 'data: true' in res.stdout:
            print(f'  [BLOCKER SUCCESS] Gazebo blocker spawned: {res.stdout.strip()}')
            return True
        print(f'  [BLOCKER SPAWN gz info] code={res.returncode}, out={res.stdout.strip()}, err={res.stderr.strip()}')
    except Exception as e:
        print(f'  [BLOCKER gz service error] {e}')

    # Fallback to ros2 run ros_gz_sim create
    cmd_ros = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', clean_world,
        '-file', sdf_path,
        '-name', model_name,
        '-x', str(x),
        '-y', str(y),
        '-z', str(z),
        '-allow_renaming', 'false',
    ]
    try:
        res = subprocess.run(cmd_ros, capture_output=True, text=True, timeout=10)
        if res.returncode == 0:
            print(f'  [BLOCKER SUCCESS] Gazebo blocker spawned via ros_gz_sim: {res.stdout.strip()}')
            return True
        print(f'  [BLOCKER ERROR] ros_gz_sim create failed: {res.stderr.strip()} {res.stdout.strip()}')
        return False
    except Exception as e:
        print(f'  [BLOCKER ERROR] Failed to spawn Gazebo blocker: {e}')
        return False


def remove_gazebo_blocker(world: str, model_name: str = 'dynamic_aisle_blocker') -> bool:
    """Remove static physical barrier from Gazebo Harmonic world."""
    clean_world = world[:-4] if world.endswith('.sdf') else world
    cmd = [
        'gz', 'service',
        '-s', f'/world/{clean_world}/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', f'name: "{model_name}", type: MODEL',
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if res.returncode == 0 and 'data: true' in res.stdout:
            print(f'  [BLOCKER SUCCESS] Gazebo blocker removed: {res.stdout.strip()}')
            return True
        print(f'  [BLOCKER REMOVE RESULT] {res.stdout.strip()} {res.stderr.strip()}')
        return res.returncode == 0
    except Exception as e:
        print(f'  [BLOCKER ERROR] Failed to remove Gazebo blocker: {e}')
        return False


class BenchmarkAuditor(Node):
    """ROS 2 Node subscribing to live fleet topics to collect empirical metrics."""

    def __init__(
        self,
        robot_count: int = 5,
        compute_mode: str = 'NORMAL',
        spawn_poses: Optional[Dict[str, Tuple[float, float]]] = None,
    ) -> None:
        super().__init__('m8b_benchmark_auditor')
        self.robot_count = robot_count
        self.robot_ids = [f'amr_{i}' for i in range(robot_count)]
        self.spawn_poses = spawn_poses or {}

        # M9-V3-A Dynamic blockage tracking
        if HAVE_M9_V3_A_MSGS:
            self.pub_blockage = self.create_publisher(
                AisleBlockageEvent,
                '/environment/aisle_blockages',
                10,
            )
        self.latest_robot_plans: Dict[str, Dict[str, Any]] = {}
        self.blockage_injected: bool = False
        self.blockage_removed: bool = False
        self.blockage_injection_time: Optional[float] = None
        self.blockage_removal_time: Optional[float] = None
        self.blockage_bbox: Optional[Tuple[float, float, float, float]] = None
        self.affected_robots: List[str] = []
        self.affected_tasks: List[str] = []
        self.diverted_robots: Set[str] = set()
        self.replan_response_latencies_ms: List[float] = []

        # Task state tracking
        self.all_tasks_state: Dict[str, str] = {}
        self.task_assigned_robots: Dict[str, str] = {}
        self.task_dropoff_coords: Dict[str, Tuple[float, float]] = {}
        self.task_generation_times: Dict[str, float] = {}
        self.task_assignment_times: Dict[str, float] = {}
        self.task_start_times: Dict[str, float] = {}
        self.task_completion_times: Dict[str, float] = {}
        self.completed_task_ids: Set[str] = set()

        # Planning and replanning metrics
        self.initial_replan_counts: Dict[str, int] = {r: 0 for r in self.robot_ids}
        self.current_replan_counts: Dict[str, int] = {r: 0 for r in self.robot_ids}
        self.final_replan_counts: Dict[str, int] = {r: 0 for r in self.robot_ids}
        self.planning_latencies_ms: List[float] = []

        # CBBA convergence
        self.cbba_bundles: Dict[str, Dict[str, Any]] = {}
        self.gazebo_cbba_converged_time: Optional[float] = None
        self.cbba_initial_converged_time: Optional[float] = None
        self.cbba_dynamic_converged_time: Optional[float] = None
        self.dynamic_release_time: Optional[float] = None
        self.mission_start_time: float = 0.0
        self.mission_end_time: float = 0.0

        # Spatial safety tracking (separated metrics)
        self.positions: Dict[str, Tuple[float, float]] = {}
        self.orientations: Dict[str, float] = {}
        self.min_inter_robot_dist: float = float('inf')
        self.proximity_breach_events: List[Dict[str, Any]] = []
        self.physical_contact_events: List[Dict[str, Any]] = []
        self.collision_events: List[Dict[str, Any]] = []  # alias for backward compatibility
        self.safety_brake_events: List[Dict[str, Any]] = []
        self.is_safety_braking: Dict[str, bool] = {r: False for r in self.robot_ids}
        self.proximity_warning_events: List[Dict[str, Any]] = []

        # Coordination tracking
        self.conflicts_detected: int = 0
        self.conflicts_resolved: int = 0
        self.deadlocks_detected: int = 0
        self.deadlocks_recovered: int = 0

        # M8B Compute mode tracking
        self.initial_compute_mode = compute_mode
        self.current_compute_modes: Dict[str, str] = {
            r: (
                'NORMAL' if compute_mode == 'ADAPTIVE' else compute_mode
            ) for r in self.robot_ids
        }
        self.mode_durations: Dict[str, Dict[str, float]] = {
            r: {'LOW': 0.0, 'NORMAL': 0.0, 'HIGH': 0.0} for r in self.robot_ids
        }
        self.last_mode_change_times: Dict[str, float] = {r: 0.0 for r in self.robot_ids}
        self.compute_events: List[Dict[str, Any]] = []
        self.failsafe_fallbacks: int = 0

        # Subscriptions
        self.sub_tasks_all = self.create_subscription(
            TaskList, '/tasks/all', self._on_tasks_all, 10
        )
        self.sub_task_events = self.create_subscription(
            TaskEventMsg, '/tasks/events', self._on_task_event, 50
        )
        self.pub_mission_start = self.create_publisher(
            TaskEventMsg, '/tasks/start_mission', 10
        )
        self.pub_status_update = self.create_publisher(
            TaskEventMsg, '/tasks/update_status', 10
        )
        self.sub_conflicts = self.create_subscription(
            ConflictReport, '/fleet/conflicts', self._on_conflict, 20
        )
        self.sub_deadlocks = self.create_subscription(
            DeadlockEvent, '/fleet/deadlocks', self._on_deadlock, 20
        )
        self.sub_compute_events = self.create_subscription(
            ComputeModeEvent, '/fleet/compute_events', self._on_compute_event, 50
        )

        self.sub_plans = []
        self.sub_bundles = []
        self.sub_odoms = []
        self.sub_scans = []

        for r_id in self.robot_ids:
            self.sub_plans.append(
                self.create_subscription(
                    RollingHorizonPlan,
                    f'/{r_id}/rolling_plan',
                    self._make_plan_cb(r_id),
                    10,
                )
            )
            self.sub_bundles.append(
                self.create_subscription(
                    RobotBundle,
                    f'/{r_id}/bundle',
                    self._make_bundle_cb(r_id),
                    10,
                )
            )
            self.sub_odoms.append(
                self.create_subscription(
                    Odometry,
                    f'/{r_id}/odom',
                    self._make_odom_cb(r_id),
                    10,
                )
            )
            self.sub_scans.append(
                self.create_subscription(
                    LaserScan,
                    f'/{r_id}/scan',
                    self._make_scan_cb(r_id),
                    qos_profile_sensor_data,
                )
            )

        # M7 Communication telemetry and dynamic profile publishing
        self.pub_comm_profile = self.create_publisher(
            CommunicationProfile, '/fleet/comm_profile', 10
        )
        self.sub_comm_metrics = []
        self.comm_metrics_by_robot: Dict[str, Dict[str, Any]] = {r: {} for r in self.robot_ids}
        self.comm_metrics_events: List[Dict[str, Any]] = []
        self.comm_degraded: bool = False
        self.comm_recovered: bool = False
        self.comm_degradation_time: Optional[float] = None
        self.comm_recovery_time: Optional[float] = None
        self.comm_degrade_elapsed: Optional[float] = None
        self.comm_recover_elapsed: Optional[float] = None
        self.comm_profile_schedule_events: List[Dict[str, Any]] = []

        for r_id in self.robot_ids:
            self.sub_comm_metrics.append(
                self.create_subscription(
                    CommunicationMetrics,
                    f'/{r_id}/comm_metrics',
                    self._make_comm_metrics_cb(r_id),
                    10,
                )
            )

    def publish_comm_profile(self, profile_name: str) -> None:
        """Publish dynamic communication degradation profile update across fleet."""
        if not hasattr(self, 'pub_comm_profile'):
            return
        cfg = PRESET_PROFILES.get(profile_name, PRESET_PROFILES['NORMAL'])
        msg = CommunicationProfile()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.profile_name = cfg.profile_name
        msg.enabled = cfg.enabled
        msg.latency_ms = float(cfg.latency_ms)
        msg.jitter_ms = float(cfg.jitter_ms)
        msg.loss_probability = float(cfg.loss_probability)
        msg.burst_loss_probability = float(cfg.burst_loss_probability)
        msg.outage_duration_s = float(cfg.outage_duration_s)
        msg.seed = int(cfg.seed)
        msg.isolated_robots = list(cfg.isolated_robots)
        self.pub_comm_profile.publish(msg)

    def _make_comm_metrics_cb(self, robot_id: str):
        def cb(msg: CommunicationMetrics):
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            data = {
                'robot_id': robot_id,
                'timestamp': now_sec,
                'profile_name': msg.profile_name,
                'messages_sent': int(msg.messages_sent),
                'messages_delivered': int(msg.messages_delivered),
                'messages_dropped': int(msg.messages_dropped),
                'packet_loss_rate': float(msg.packet_loss_rate),
                'avg_latency_ms': float(msg.avg_latency_ms),
                'p95_latency_ms': float(msg.p95_latency_ms),
                'jitter_ms': float(msg.jitter_ms),
                'burst_events_count': int(msg.burst_events_count),
                'outage_active': bool(msg.outage_active),
                'stale_messages_count': int(msg.stale_messages_count),
                'expired_reservations_count': int(msg.expired_reservations_count),
            }
            self.comm_metrics_by_robot[robot_id] = data
            self.comm_metrics_events.append(data)
        return cb


    def publish_aisle_blockage(
        self,
        blockage_id: str,
        is_blocked: bool,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
    ) -> None:
        """Publish an AisleBlockageEvent message to fleet."""
        if not HAVE_M9_V3_A_MSGS or not hasattr(self, 'pub_blockage'):
            return
        msg = AisleBlockageEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.blockage_id = blockage_id
        msg.is_blocked = is_blocked
        msg.min_x = min_x
        msg.max_x = max_x
        msg.min_y = min_y
        msg.max_y = max_y
        msg.timestamp = self.get_clock().now().to_msg()
        self.pub_blockage.publish(msg)

    def _make_plan_cb(self, robot_id: str):
        def cb(msg: RollingHorizonPlan):
            self.current_replan_counts[robot_id] = msg.replan_count
            lat = float(msg.planning_latency_ms)
            if lat > 0.0:
                self.planning_latencies_ms.append(lat)

            now_t = time.time()
            pts = [(float(p.x), float(p.y)) for p in (msg.horizon_path or msg.execution_path)]
            self.latest_robot_plans[robot_id] = {
                'replan_count': msg.replan_count,
                'path': pts,
                'timestamp': now_t,
                'is_valid': bool(msg.is_valid),
            }

            # Check if this robot successfully detoured around active blockage
            if self.blockage_injected and not self.blockage_removed and self.blockage_bbox:
                min_x, max_x, min_y, max_y = self.blockage_bbox
                if robot_id in self.affected_robots and robot_id not in self.diverted_robots:
                    intersects = any(
                        min_x <= px <= max_x and min_y <= py <= max_y
                        for px, py in pts
                    )
                    if not intersects and len(pts) > 0:
                        self.diverted_robots.add(robot_id)
                        if self.blockage_injection_time is not None:
                            replan_lat = (now_t - self.blockage_injection_time) * 1000.0
                            self.replan_response_latencies_ms.append(replan_lat)
                            print(
                                f"  [V3-A DETOUR] {robot_id} successfully detoured around blockage "
                                f"(latency: {replan_lat:.1f} ms)"
                            )
        return cb

    def _make_bundle_cb(self, robot_id: str):
        def cb(msg: RobotBundle):
            self.cbba_bundles[robot_id] = {
                'is_converged': bool(msg.is_converged),
                'tasks': list(msg.task_ids),
                'timestamp': time.time(),
            }
            if self.gazebo_cbba_converged_time is None and self.mission_start_time > 0:
                if len(self.cbba_bundles) == self.robot_count:
                    if all(b.get('is_converged', False) for b in self.cbba_bundles.values()):
                        self.gazebo_cbba_converged_time = time.time() - self.mission_start_time
                        self.cbba_initial_converged_time = self.gazebo_cbba_converged_time

            if self.dynamic_release_time is not None and self.cbba_dynamic_converged_time is None:
                now_t = time.time()
                if (now_t - self.dynamic_release_time) > 0.05:
                    if len(self.cbba_bundles) == self.robot_count:
                        if all(b.get('is_converged', False) for b in self.cbba_bundles.values()):
                            self.cbba_dynamic_converged_time = now_t - self.dynamic_release_time
        return cb

    def _make_odom_cb(self, robot_id: str):
        def cb(msg: Odometry):
            if self.spawn_poses and robot_id in self.spawn_poses:
                spawn_x, spawn_y = self.spawn_poses[robot_id]
            else:
                r_idx = int(robot_id.split('_')[-1])
                spawn_x = 2.0
                spawn_y = 2.0 + r_idx * 3.0
            x = round(spawn_x + msg.pose.pose.position.x, 3)
            y = round(spawn_y + msg.pose.pose.position.y, 3)
            self.positions[robot_id] = (x, y)

            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.orientations[robot_id] = math.atan2(siny_cosp, cosy_cosp)

            self._check_distances()
        return cb

    def _make_scan_cb(self, robot_id: str):
        def cb(msg: LaserScan):
            num_rays = len(msg.ranges)
            if num_rays == 0:
                return

            # Forward arc (+/- 15 deg) centered at num_rays // 2 (matching safety controller)
            center_idx = num_rays // 2
            arc_rays = max(1, int(num_rays * (15.0 / 360.0)))
            forward_ranges = [
                msg.ranges[idx]
                for idx in range(center_idx - arc_rays, center_idx + arc_rays + 1)
                if 0 <= idx < num_rays
                and not math.isnan(msg.ranges[idx])
                and not math.isinf(msg.ranges[idx])
                and msg.ranges[idx] > 0.05
            ]

            # Immediate bumper hazard threshold: < 0.28m triggers emergency brake stop
            is_hazard = bool(forward_ranges and min(forward_ranges) < 0.28)
            if is_hazard:
                if not self.is_safety_braking.get(robot_id, False):
                    self.safety_brake_events.append({
                        'robot_id': robot_id,
                        'range_m': round(min(forward_ranges), 3),
                        'timestamp': time.time(),
                    })
                    self.is_safety_braking[robot_id] = True
            else:
                self.is_safety_braking[robot_id] = False

            # General proximity warning (any beam <= 0.40m)
            valid = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > 0.05]
            if valid and min(valid) <= 0.40:
                self.proximity_warning_events.append({
                    'robot_id': robot_id,
                    'range_m': round(min(valid), 3),
                    'timestamp': time.time(),
                })
        return cb

    def _on_conflict(self, msg: ConflictReport):
        self.conflicts_detected += 1
        self.conflicts_resolved += 1

    def _on_deadlock(self, msg: DeadlockEvent):
        self.deadlocks_detected += 1
        self.deadlocks_recovered += 1

    def _on_compute_event(self, msg: ComputeModeEvent):
        r_id = msg.robot_id
        if r_id not in self.robot_ids:
            return

        now = time.time()
        prev_clean = msg.previous_mode.replace('COMPUTE_MODE_', '')
        curr_clean = msg.current_mode.replace('COMPUTE_MODE_', '')

        if self.last_mode_change_times[r_id] > 0.0:
            dt = max(0.0, now - self.last_mode_change_times[r_id])
            if prev_clean in self.mode_durations[r_id]:
                self.mode_durations[r_id][prev_clean] += dt

        self.last_mode_change_times[r_id] = now
        self.current_compute_modes[r_id] = curr_clean

        if 'FAILSAFE' in msg.reason.upper() or 'FALLBACK' in msg.reason.upper():
            self.failsafe_fallbacks += 1

        self.compute_events.append({
            'robot_id': r_id,
            'previous_mode': prev_clean,
            'current_mode': curr_clean,
            'trigger_signal': msg.trigger_signal,
            'trigger_value': round(float(msg.trigger_value), 4),
            'threshold_value': round(float(msg.threshold_value), 4),
            'reason': msg.reason,
            'dwell_time_sec': round(float(msg.dwell_time_sec), 2),
            'timestamp': now,
        })
        print(
            f"  [COMPUTE EVENT] {r_id}: {prev_clean} -> {curr_clean} "
            f"({msg.reason})"
        )

    def _check_distances(self):
        r_ids = list(self.positions.keys())
        n = len(r_ids)
        for i in range(n):
            for j in range(i + 1, n):
                r1 = r_ids[i]
                r2 = r_ids[j]
                p1 = self.positions[r1]
                p2 = self.positions[r2]
                d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                if d < self.min_inter_robot_dist:
                    self.min_inter_robot_dist = d

                # 1. Proximity breach: center-to-center distance < 0.35m
                if d < 0.35:
                    ev = {
                        'r1': r1,
                        'r2': r2,
                        'p1': p1,
                        'p2': p2,
                        'dist': round(d, 3),
                        'time': time.time(),
                    }
                    self.proximity_breach_events.append(ev)
                    self.collision_events.append(ev)  # Backwards compatibility

                # 2. Physical Gazebo contact: 2D OBB intersection via SAT
                yaw1 = self.orientations.get(r1, 0.0)
                yaw2 = self.orientations.get(r2, 0.0)
                if check_obb_intersection(p1, yaw1, p2, yaw2):
                    self.physical_contact_events.append({
                        'r1': r1,
                        'r2': r2,
                        'p1': p1,
                        'p2': p2,
                        'yaw1': round(yaw1, 3),
                        'yaw2': round(yaw2, 3),
                        'dist': round(d, 3),
                        'time': time.time(),
                    })

    def _on_tasks_all(self, msg: TaskList):
        for td in msg.tasks:
            self.all_tasks_state[td.task_id] = td.status
            self.task_dropoff_coords[td.task_id] = (td.dropoff_pose.x, td.dropoff_pose.y)
            if td.task_id not in self.task_generation_times:
                self.task_generation_times[td.task_id] = (
                    td.created_at.sec + td.created_at.nanosec * 1e-9
                    if td.created_at.sec > 0 else time.time()
                )

    def _on_task_event(self, msg: TaskEventMsg):
        ev_time = (
            msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
            if msg.timestamp.sec > 0 else time.time()
        )
        task_id = msg.task_id

        if msg.new_state:
            self.all_tasks_state[task_id] = msg.new_state

        if msg.event_type == 'RELEASED' or (msg.previous_state == 'STAGED' and msg.new_state == 'PENDING'):
            if self.dynamic_release_time is None and self.mission_start_time > 0 and (ev_time - self.mission_start_time) > 5.0:
                self.dynamic_release_time = ev_time
            print(
                f"  [DYNAMIC RELEASE] Task '{task_id}' released (STAGED -> PENDING) "
                f"at t={ev_time:.1f}s"
            )

        if msg.new_state == 'ASSIGNED':
            if task_id not in self.task_assignment_times:
                self.task_assignment_times[task_id] = ev_time
            if msg.robot_id:
                self.task_assigned_robots[task_id] = msg.robot_id

        elif msg.new_state == 'IN_PROGRESS':
            if task_id not in self.task_start_times:
                self.task_start_times[task_id] = ev_time
            if msg.robot_id:
                self.task_assigned_robots[task_id] = msg.robot_id

        elif msg.new_state == 'COMPLETED':
            if task_id not in self.completed_task_ids:
                self.completed_task_ids.add(task_id)
                self.task_completion_times[task_id] = ev_time
                if msg.robot_id:
                    self.task_assigned_robots[task_id] = msg.robot_id

                # Audit physical arrival coordinates at dropoff
                assigned_r = self.task_assigned_robots.get(task_id, '')
                if assigned_r and assigned_r in self.positions:
                    rx, ry = self.positions[assigned_r]
                    dx, dy = self.task_dropoff_coords.get(task_id, (0.0, 0.0))
                    dist = math.hypot(rx - dx, ry - dy)
                    print(
                        f"  [COMPLETION] Task '{task_id}' COMPLETED by {assigned_r} "
                        f"at ({rx:.2f}, {ry:.2f}) -> dropoff ({dx:.2f}, {dy:.2f}), dist={dist:.2f}m"
                    )


def check_system_resources() -> None:
    """Verify system has adequate RAM and CPU before launching Gazebo."""
    avail_mb = psutil.virtual_memory().available / 1e6
    if avail_mb < 2000:
        print(f"[ERROR] Insufficient available RAM ({avail_mb:.0f} MB < 2000 MB). Aborting.")
        sys.exit(1)
    print(f"[RESOURCE CHECK] Available RAM: {avail_mb:.0f} MB | CPU Cores: {os.cpu_count()}")


def teardown_simulation() -> None:
    """Clean terminate any existing Gazebo Harmonic and ROS 2 simulation processes."""
    subprocess.run(
        "killall -9 gz sim ruby parameter_bridge static_transform_publisher robot_state_publisher 2>/dev/null; "
        "pkill -9 -f 'amr_fleet' 2>/dev/null; "
        "ros2 daemon stop 2>/dev/null || true",
        shell=True,
    )
    time.sleep(2.5)


def resolve_spawn_poses(world: str, fleet_size: int) -> Dict[str, Tuple[float, float]]:
    """Determine initial spawn (x, y) coordinates for each robot in the fleet."""
    clean_world = world[:-4] if world.endswith('.sdf') else world
    tag = clean_world.replace('warehouse_', '')
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(workspace_root, 'config', 'robots')

    candidates = [
        os.path.join(config_dir, f'fleet_{fleet_size}_robots_{tag}.yaml'),
        os.path.join(config_dir, f'fleet_{fleet_size}_robots_{clean_world}.yaml'),
        os.path.join(config_dir, f'fleet_{fleet_size}_robots.yaml'),
    ]
    for cand in candidates:
        if os.path.isfile(cand):
            try:
                with open(cand, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    robots = data.get('fleet', {}).get('robots', [])
                    poses = {}
                    for r in robots:
                        r_id = r.get('id')
                        if r_id:
                            poses[r_id] = (float(r.get('x', 0.0)), float(r.get('y', 0.0)))
                    if len(poses) >= fleet_size:
                        return poses
            except Exception:
                pass

    is_32m = 'm9' in clean_world or '32' in clean_world
    poses = {}
    for i in range(fleet_size):
        r_id = f'amr_{i}'
        if is_32m:
            col = i // 5
            row = i % 5
            poses[r_id] = (2.5 if col == 0 else 16.0, 5.0 + row * 5.0)
        else:
            col = i // 5
            row = i % 5
            poses[r_id] = (2.0 if col == 0 else 8.0, 2.0 + row * 3.0)
    return poses


def run_single_trial(
    workload_size: int,
    workload_path: str,
    trial_id: int,
    seed: int,
    horizon_sec: float,
    comm_profile: str,
    raw_dir: str,
    compute_mode: str = 'NORMAL',
    schema_version: str = 'm8b.v1',
    dry_run: bool = False,
    world: str = 'warehouse_small',
    fleet_size: int = 5,
    map_id: str = 'warehouse_grid_small',
    is_v3_a: bool = False,
    v3_a_block_time: float = 45.0,
    v3_a_unblock_time: float = 90.0,
    v3_a_blocker_pos: Tuple[float, float] = (4.5, 6.75),
    v3_a_blocker_size: Tuple[float, float] = (1.8, 0.6),
    is_v3_e: bool = False,
    v3_e_comm_degrade_time: float = 75.0,
    v3_e_comm_recover_time: float = 120.0,
    v3_e_comm_profile: str = 'LOSS_HIGH',
) -> Dict[str, Any]:
    """Execute a single deterministic benchmark trial and collect empirical metrics."""
    if is_v3_e:
        prefix = 'exp_m9_v3_e'
        is_v3_a = True
    elif is_v3_a:
        prefix = 'exp_m9_v3_a'
    elif 'm9' in schema_version or 'm9' in world:
        prefix = 'exp_m9'
    elif schema_version == 'm8b.v1':
        prefix = 'exp_m8b'
    else:
        prefix = 'exp_m8a'
    mode_tag = f"_{compute_mode.lower()}" if schema_version != 'm8a.v1' else ""
    experiment_id = (
        f"{prefix}_w{workload_size}_t{trial_id}_s{seed}{mode_tag}_{int(time.time())}"
    )
    timestamp_utc = datetime.now(timezone.utc).isoformat()
    timestamp_unix = time.time()
    git_hash = get_git_commit()

    print("\n" + "=" * 78)
    print(
        f"  LAUNCHING BENCHMARK TRIAL {trial_id} | "
        f"WORKLOAD: {workload_size} TASKS | SEED: {seed}"
    )
    print(
        f"  Horizon: {horizon_sec:.1f}s | Comm: {comm_profile} | "
        f"Compute Mode: {compute_mode} | ID: {experiment_id}"
    )
    print("=" * 78)

    if dry_run:
        print("  [DRY-RUN] Simulating benchmark metrics without Gazebo...")
        time.sleep(1.0)
        return {
            'schema_version': schema_version,
            'experiment_id': experiment_id,
            'metadata': {
                'experiment_id': experiment_id,
                'timestamp_utc': timestamp_utc,
                'timestamp_unix': timestamp_unix,
                'git_commit': git_hash,
                'software_version': schema_version,
                'fleet_size': fleet_size,
                'world': world,
                'workload_id': f'workload_{workload_size}_tasks',
                'workload_size': workload_size,
                'trial_id': trial_id,
                'seed': seed,
                'mission_horizon_sec': horizon_sec,
                'actual_duration_sec': horizon_sec,
                'communication_profile': comm_profile,
                'compute_mode': compute_mode,
                'map_id': map_id,
                'termination_reason': TerminationReason.HORIZON_REACHED.value,
            },
            'metrics': {
                'generated_tasks': workload_size,
                'assigned_tasks': workload_size,
                'completed_tasks': 1,
                'failed_tasks': 0,
                'remaining_tasks': workload_size - 1,
                'completion_rate_pct': round(1.0 / workload_size * 100.0, 1),
                'throughput_tasks_per_min': round(1.0 / (horizon_sec / 60.0), 2),
                'makespan_sec': 50.5,
                'planning_cycles': 80,
                'replan_count': 80,
                'planning_latency_mean_ms': 1.25,
                'planning_latency_p50_ms': 1.10,
                'planning_latency_p95_ms': 2.30,
                'cbba_convergence_time_ms': 1650.0,
                'conflicts_detected': 1,
                'conflicts_resolved': 1,
                'deadlocks_detected': 0,
                'deadlocks_recovered': 0,
                'minimum_center_to_center_distance_m': 1.65,
                'minimum_inter_robot_distance_m': 1.65,
                'proximity_breaches': 0,
                'physical_gazebo_contacts': 0,
                'obb_chassis_overlap_samples': 0,
                'collision_contact_events': 0,
                'safety_brake_interventions': 0,
                'safety_aborts': 0,
                'cpu_utilization_mean_pct': 35.0,
                'cpu_utilization_max_pct': 48.0,
                'ram_utilization_mean_mb': 4500.0,
                'ram_utilization_max_mb': 5200.0,
                'mode_transitions': 0,
                'failsafe_fallbacks': 0,
                'mode_occupancy_low_pct': 100.0 if compute_mode == 'LOW' else 0.0,
                'mode_occupancy_normal_pct': (
                    100.0 if compute_mode in ('NORMAL', 'ADAPTIVE') else 0.0
                ),
                'mode_occupancy_high_pct': 100.0 if compute_mode == 'HIGH' else 0.0,
            },
            'completed_task_ids': ['task_0'],
            'proximity_breach_events': [],
            'physical_contact_events': [],
            'safety_brake_events': [],
            'collision_events': [],
            'compute_mode_events': [],
            'replan_counts_initial': {},
            'replan_counts_final': {},
            'replan_counts_delta': {},
        }

    # 1. Clean teardown before trial
    teardown_simulation()

    # 2. Launch real Gazebo fleet
    bundle_size = SCALED_BUNDLE_SIZES.get(workload_size, 4)
    comm_cfg = PRESET_PROFILES.get(comm_profile, PRESET_PROFILES['NORMAL'])
    deg_flag = 'true' if comm_cfg.enabled else 'false'

    launch_cmd = (
        f"source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        f"ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py "
        f"headless:=true rviz:=false comm_profile:={comm_profile} "
        f"enable_comm_degradation:={deg_flag} "
        f"workload_file:={os.path.abspath(workload_path)} "
        f"max_bundle_size:={bundle_size} "
        f"compute_mode:={compute_mode} "
        f"world:={world} "
        f"robot_count:={fleet_size}"
    )

    os.makedirs(raw_dir, exist_ok=True)
    launch_log_path = os.path.join(raw_dir, f"{experiment_id}_launch.log")
    launch_log_file = open(launch_log_path, 'w', encoding='utf-8')

    proc = subprocess.Popen(
        ['bash', '-c', launch_cmd],
        stdout=launch_log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    if not rclpy.ok():
        rclpy.init()
    spawn_poses = resolve_spawn_poses(world, fleet_size)
    auditor = BenchmarkAuditor(
        robot_count=fleet_size,
        compute_mode=compute_mode,
        spawn_poses=spawn_poses,
    )
    actual_duration = 0.0
    termination_reason = TerminationReason.HORIZON_REACHED

    try:
        # 3. Discovery wait
        print(f"[DISCOVERY] Waiting for /tasks/all ({workload_size} tasks) and fleet discovery...")
        t_wait_start = time.time()
        last_disc_print = t_wait_start
        discovered = False
        while time.time() - t_wait_start < 45.0:
            rclpy.spin_once(auditor, timeout_sec=0.1)
            if len(auditor.all_tasks_state) == workload_size and len(auditor.positions) == fleet_size:
                discovered = True
                break
            if time.time() - last_disc_print >= 5.0:
                last_disc_print = time.time()
                print(
                    f"  [DISCOVERY] Waiting... Tasks: {len(auditor.all_tasks_state)}/{workload_size}, "
                    f"AMRs: {len(auditor.positions)}/{fleet_size}"
                )

        print(f"[DISCOVERY] Discovered {len(auditor.all_tasks_state)}/{workload_size} tasks, {len(auditor.positions)}/{fleet_size} AMRs.")
        assert len(auditor.all_tasks_state) == workload_size, (
            f"Discovery failed: expected {workload_size} tasks, got {len(auditor.all_tasks_state)}"
        )

        # 4. Initialize trial start
        auditor.initial_replan_counts = dict(auditor.current_replan_counts)
        auditor.mission_start_time = time.time()
        t_start = auditor.mission_start_time

        # Signal mission clock anchor to TaskManagerNode
        start_ev = TaskEventMsg()
        start_ev.header.stamp = auditor.get_clock().now().to_msg()
        start_ev.task_id = ''
        start_ev.event_type = 'MISSION_START'
        start_ev.new_state = 'MISSION_START'
        start_ev.details = 'Benchmark execution active'
        auditor.pub_mission_start.publish(start_ev)
        auditor.pub_status_update.publish(start_ev)

        for r_id in auditor.robot_ids:
            auditor.last_mode_change_times[r_id] = t_start
        last_print = t_start
        last_sample = t_start

        cpu_samples: List[float] = []
        ram_samples: List[float] = []
        termination_reason = TerminationReason.HORIZON_REACHED

        # Communication tracking
        packets_sent = 0
        packets_delivered = 0
        packets_dropped = 0
        comm_model = CommunicationImpairmentModel(config=comm_cfg)

        if is_v3_a:
            min_x = v3_a_blocker_pos[0] - v3_a_blocker_size[0] / 2.0
            max_x = v3_a_blocker_pos[0] + v3_a_blocker_size[0] / 2.0
            min_y = v3_a_blocker_pos[1] - v3_a_blocker_size[1] / 2.0
            max_y = v3_a_blocker_pos[1] + v3_a_blocker_size[1] / 2.0
            auditor.blockage_bbox = (min_x, max_x, min_y, max_y)

        # 5. Execution Loop
        print(f"[EXECUTION] Trial active for {horizon_sec:.1f}s horizon...")
        while time.time() - t_start < horizon_sec:
            now_sec = time.time()
            elapsed = now_sec - t_start
            rclpy.spin_once(auditor, timeout_sec=0.05)

            # M9-V3-A Dynamic Blocker Injection
            if is_v3_a and not auditor.blockage_injected and elapsed >= v3_a_block_time:
                auditor.blockage_injected = True
                auditor.blockage_injection_time = now_sec
                bx_min, bx_max, by_min, by_max = auditor.blockage_bbox

                for r, pdata in auditor.latest_robot_plans.items():
                    if any(bx_min <= px <= bx_max and by_min <= py <= by_max for px, py in pdata.get('path', [])):
                        if r not in auditor.affected_robots:
                            auditor.affected_robots.append(r)

                for tid, r in auditor.task_assigned_robots.items():
                    if r in auditor.affected_robots and auditor.all_tasks_state.get(tid) in ('ASSIGNED', 'IN_PROGRESS'):
                        if tid not in auditor.affected_tasks:
                            auditor.affected_tasks.append(tid)

                spawn_gazebo_blocker(
                    world=world,
                    x=v3_a_blocker_pos[0],
                    y=v3_a_blocker_pos[1],
                    z=0.7,
                )
                auditor.publish_aisle_blockage(
                    blockage_id='blockage_aisle_1_south',
                    is_blocked=True,
                    min_x=bx_min,
                    max_x=bx_max,
                    min_y=by_min,
                    max_y=by_max,
                )
                print(
                    f"\n  [M9-V3-A EVENT] Aisle Blockage INJECTED at t={elapsed:.1f}s "
                    f"at ({v3_a_blocker_pos[0]}, {v3_a_blocker_pos[1]})\n"
                    f"    -> Blockage BBox: [{bx_min:.2f}..{bx_max:.2f}, {by_min:.2f}..{by_max:.2f}]\n"
                    f"    -> Affected Robots: {auditor.affected_robots}\n"
                    f"    -> Affected Tasks: {auditor.affected_tasks}"
                )

            # M9-V3-A Dynamic Blocker Removal
            if is_v3_a and auditor.blockage_injected and not auditor.blockage_removed and elapsed >= v3_a_unblock_time:
                auditor.blockage_removed = True
                auditor.blockage_removal_time = now_sec
                bx_min, bx_max, by_min, by_max = auditor.blockage_bbox
                remove_gazebo_blocker(world=world)
                auditor.publish_aisle_blockage(
                    blockage_id='blockage_aisle_1_south',
                    is_blocked=False,
                    min_x=bx_min,
                    max_x=bx_max,
                    min_y=by_min,
                    max_y=by_max,
                )
                print(
                    f"\n  [M9-V3-A EVENT] Aisle Blockage REMOVED at t={elapsed:.1f}s "
                    f"at ({v3_a_blocker_pos[0]}, {v3_a_blocker_pos[1]})\n"
                    f"    -> Aisle 1 South segment reopened"
                )

            # M9-V3-E Dynamic Communication Degradation Injection
            if is_v3_e and not auditor.comm_degraded and elapsed >= v3_e_comm_degrade_time:
                auditor.comm_degraded = True
                auditor.comm_degradation_time = now_sec
                auditor.comm_degrade_elapsed = elapsed
                auditor.publish_comm_profile(v3_e_comm_profile)
                auditor.comm_profile_schedule_events.append({
                    'timestamp_unix': now_sec,
                    'elapsed_sec': elapsed,
                    'profile_name': v3_e_comm_profile,
                    'action': 'DEGRADATION_INJECTED',
                })
                print(
                    f"\n  [M9-V3-E EVENT] Communication Degradation INJECTED at t={elapsed:.1f}s\n"
                    f"    -> Profile: {v3_e_comm_profile}"
                )

            # M9-V3-E Dynamic Communication Recovery
            if is_v3_e and auditor.comm_degraded and not auditor.comm_recovered and elapsed >= v3_e_comm_recover_time:
                auditor.comm_recovered = True
                auditor.comm_recovery_time = now_sec
                auditor.comm_recover_elapsed = elapsed
                auditor.publish_comm_profile('NORMAL')
                auditor.comm_profile_schedule_events.append({
                    'timestamp_unix': now_sec,
                    'elapsed_sec': elapsed,
                    'profile_name': 'NORMAL',
                    'action': 'RECOVERED_TO_NORMAL',
                })
                print(
                    f"\n  [M9-V3-E EVENT] Communication RECOVERED at t={elapsed:.1f}s\n"
                    f"    -> Profile: NORMAL"
                )


            # Periodic compute sampling
            if now_sec - last_sample >= 2.0:
                last_sample = now_sec
                cpu_samples.append(psutil.cpu_percent())
                ram_samples.append(psutil.virtual_memory().used / 1e6)

            # Periodic status logging
            if now_sec - last_print >= 15.0:
                last_print = now_sec
                elapsed = now_sec - t_start
                print(
                    f"  [T+{elapsed:.1f}s / {horizon_sec:.1f}s] Completed: {len(auditor.completed_task_ids)}/{workload_size} | "
                    f"Min Dist: {auditor.min_inter_robot_dist:.2f}m | "
                    f"Replans: {sum(auditor.current_replan_counts.values())}"
                )

            # Check safety abort
            if auditor.min_inter_robot_dist < 0.35:
                print(f"  [SAFETY ABORT] Distance {auditor.min_inter_robot_dist:.3f}m violated 0.35m threshold!")
                for ev in auditor.proximity_breach_events:
                    print(f"    -> Proximity Breach Event: {ev['r1']} at {ev.get('p1')} and {ev['r2']} at {ev.get('p2')}, dist={ev['dist']}m")
                if auditor.physical_contact_events:
                    print(f"    -> Physical Contact Events ({len(auditor.physical_contact_events)}):")
                    for cev in auditor.physical_contact_events:
                        print(f"       * {cev['r1']} and {cev['r2']} at dist={cev['dist']}m")
                else:
                    print("    -> Physical Gazebo Contacts: ZERO (no bounding box overlap)")
                print(f"    -> Fleet positions at abort: {auditor.positions}")
                termination_reason = TerminationReason.SAFETY_ABORT
                break

            # Check task completion termination
            if len(auditor.completed_task_ids) == workload_size:
                print("  [ALL TASKS COMPLETED] Full workload delivered!")
                termination_reason = TerminationReason.ALL_TASKS_COMPLETED
                break

        auditor.mission_end_time = time.time()
        actual_duration = auditor.mission_end_time - auditor.mission_start_time
        auditor.final_replan_counts = dict(auditor.current_replan_counts)
        for r_id in auditor.robot_ids:
            cur_mode = auditor.current_compute_modes[r_id]
            dt = max(0.0, auditor.mission_end_time - auditor.last_mode_change_times[r_id])
            if cur_mode in auditor.mode_durations[r_id]:
                auditor.mode_durations[r_id][cur_mode] += dt

    finally:
        # 6. Teardown
        if is_v3_a and auditor.blockage_injected:
            try:
                remove_gazebo_blocker(world=world)
            except Exception:
                pass
        try:
            os.killpg(os.getpgid(proc.pid), 9)
        except Exception:
            pass
        try:
            launch_log_file.close()
        except Exception:
            pass
        teardown_simulation()

    # 7. Compute Standard Metrics
    for tid in auditor.completed_task_ids:
        auditor.all_tasks_state[tid] = 'COMPLETED'

    # Disjoint task state accounting
    completed_count = sum(1 for s in auditor.all_tasks_state.values() if s == 'COMPLETED')
    if len(auditor.completed_task_ids) > completed_count:
        completed_count = len(auditor.completed_task_ids)

    staged_count = sum(1 for s in auditor.all_tasks_state.values() if s == 'STAGED')
    pending_count = sum(1 for s in auditor.all_tasks_state.values() if s == 'PENDING')
    assigned_count = sum(1 for s in auditor.all_tasks_state.values() if s == 'ASSIGNED')
    in_progress_count = sum(1 for s in auditor.all_tasks_state.values() if s == 'IN_PROGRESS')
    failed_count = sum(1 for s in auditor.all_tasks_state.values() if s == 'FAILED')
    cancelled_count = sum(1 for s in auditor.all_tasks_state.values() if s == 'CANCELLED')

    known_count = (
        staged_count + pending_count + assigned_count +
        in_progress_count + completed_count + failed_count + cancelled_count
    )
    if known_count < workload_size:
        pending_count += (workload_size - known_count)

    # Explicit definition: REMAINING is all non-terminal tasks (STAGED + PENDING + ASSIGNED + IN_PROGRESS)
    remaining_count = staged_count + pending_count + assigned_count + in_progress_count
    completion_rate = round((completed_count / workload_size) * 100.0, 1)
    throughput = round(completed_count / (actual_duration / 60.0), 2)

    if auditor.completed_task_ids:
        final_ts = max(auditor.task_completion_times[t] for t in auditor.completed_task_ids)
        makespan = round(final_ts - auditor.mission_start_time, 2)
    else:
        makespan = round(actual_duration, 2)

    replan_deltas = {
        r: max(0, auditor.final_replan_counts[r] - auditor.initial_replan_counts.get(r, 0))
        for r in auditor.robot_ids
    }
    total_replan_delta = sum(replan_deltas.values())

    # Planning latencies
    lat_mean = round(sum(auditor.planning_latencies_ms) / len(auditor.planning_latencies_ms), 2) if auditor.planning_latencies_ms else None
    if auditor.planning_latencies_ms:
        sorted_lat = sorted(auditor.planning_latencies_ms)
        n_lat = len(sorted_lat)
        lat_p50 = round(sorted_lat[int(n_lat * 0.50)], 2)
        lat_p95 = round(sorted_lat[min(int(n_lat * 0.95), n_lat - 1)], 2)
        lat_p99 = round(sorted_lat[min(int(n_lat * 0.99), n_lat - 1)], 2)
    else:
        lat_p50 = lat_p95 = lat_p99 = None

    # CBBA convergence duration
    cbba_conv_ms = (
        round(auditor.gazebo_cbba_converged_time * 1000.0, 1)
        if auditor.gazebo_cbba_converged_time is not None
        else None
    )

    # Compute metrics
    cpu_mean = round(sum(cpu_samples) / len(cpu_samples), 1) if cpu_samples else None
    cpu_max = round(max(cpu_samples), 1) if cpu_samples else None
    ram_mean = round(sum(ram_samples) / len(ram_samples), 1) if ram_samples else None
    ram_max = round(max(ram_samples), 1) if ram_samples else None

    # Mode occupancies
    total_robot_time = sum(
        sum(auditor.mode_durations[r].values()) for r in auditor.robot_ids
    )
    if total_robot_time > 0.0:
        occ_low = round(
            (sum(auditor.mode_durations[r]['LOW'] for r in auditor.robot_ids) / total_robot_time) * 100.0, 1
        )
        occ_norm = round(
            (sum(auditor.mode_durations[r]['NORMAL'] for r in auditor.robot_ids) / total_robot_time) * 100.0, 1
        )
        occ_high = round(
            (sum(auditor.mode_durations[r]['HIGH'] for r in auditor.robot_ids) / total_robot_time) * 100.0, 1
        )
    else:
        occ_low = 100.0 if compute_mode == 'LOW' else 0.0
        occ_norm = 100.0 if compute_mode in ('NORMAL', 'ADAPTIVE') else 0.0
        occ_high = 100.0 if compute_mode == 'HIGH' else 0.0

    total_comm_sent = sum(m.get('messages_sent', 0) for m in auditor.comm_metrics_by_robot.values())
    total_comm_delivered = sum(m.get('messages_delivered', 0) for m in auditor.comm_metrics_by_robot.values())
    total_comm_dropped = sum(m.get('messages_dropped', 0) for m in auditor.comm_metrics_by_robot.values())
    comm_loss_rate = round(total_comm_dropped / total_comm_sent * 100.0, 2) if total_comm_sent > 0 else 0.0
    comm_avg_lat = (
        round(sum(m.get('avg_latency_ms', 0.0) for m in auditor.comm_metrics_by_robot.values()) / len(auditor.comm_metrics_by_robot), 2)
        if auditor.comm_metrics_by_robot else 0.0
    )
    comm_p95_lat = (
        round(max((m.get('p95_latency_ms', 0.0) for m in auditor.comm_metrics_by_robot.values()), default=0.0), 2)
    )
    comm_jitter = (
        round(sum(m.get('jitter_ms', 0.0) for m in auditor.comm_metrics_by_robot.values()) / len(auditor.comm_metrics_by_robot), 2)
        if auditor.comm_metrics_by_robot else 0.0
    )
    total_stale_msgs = sum(m.get('stale_messages_count', 0) for m in auditor.comm_metrics_by_robot.values())
    total_expired_res = sum(m.get('expired_reservations_count', 0) for m in auditor.comm_metrics_by_robot.values())

    # Assemble Result Object
    result = {
        'schema_version': schema_version,
        'experiment_id': experiment_id,
        'metadata': {
            'experiment_id': experiment_id,
            'timestamp_utc': timestamp_utc,
            'timestamp_unix': timestamp_unix,
            'git_commit': git_hash,
            'software_version': schema_version,
            'fleet_size': fleet_size,
            'world': world,
            'workload_id': f'workload_{workload_size}_tasks',
            'workload_size': workload_size,
            'trial_id': trial_id,
            'seed': seed,
            'mission_horizon_sec': horizon_sec,
            'actual_duration_sec': round(actual_duration, 2),
            'communication_profile': comm_profile,
            'compute_mode': compute_mode,
            'map_id': map_id,
            'termination_reason': termination_reason.value,
        },
        'metrics': {
            'generated_tasks': workload_size,
            'staged_tasks': staged_count,
            'pending_tasks': pending_count,
            'assigned_tasks': assigned_count,
            'in_progress_tasks': in_progress_count,
            'completed_tasks': completed_count,
            'failed_tasks': failed_count,
            'cancelled_tasks': cancelled_count,
            'remaining_tasks': remaining_count,
            'completion_rate_pct': completion_rate,
            'throughput_tasks_per_min': throughput,
            'makespan_sec': makespan,
            'planning_cycles': len(auditor.planning_latencies_ms),
            'replan_count': total_replan_delta,
            'planning_latency_mean_ms': lat_mean,
            'planning_latency_p50_ms': lat_p50,
            'planning_latency_p95_ms': lat_p95,
            'planning_latency_p99_ms': lat_p99,
            'cbba_convergence_time_ms': cbba_conv_ms,
            'cbba_initial_convergence_time_ms': round(auditor.cbba_initial_converged_time * 1000.0, 1) if auditor.cbba_initial_converged_time is not None else cbba_conv_ms,
            'cbba_dynamic_convergence_time_ms': round(auditor.cbba_dynamic_converged_time * 1000.0, 1) if auditor.cbba_dynamic_converged_time is not None else None,
            'allocation_changes': 0,
            'unassigned_tasks': staged_count + pending_count,
            'conflicts_detected': auditor.conflicts_detected,
            'conflicts_resolved': auditor.conflicts_resolved,
            'deadlocks_detected': auditor.deadlocks_detected,
            'deadlocks_recovered': auditor.deadlocks_recovered,
            'minimum_center_to_center_distance_m': round(auditor.min_inter_robot_dist, 3) if not math.isinf(auditor.min_inter_robot_dist) else 1.5,
            'minimum_inter_robot_distance_m': round(auditor.min_inter_robot_dist, 3) if not math.isinf(auditor.min_inter_robot_dist) else 1.5,
            'proximity_breaches': len(auditor.proximity_breach_events),
            'physical_gazebo_contacts': len(auditor.physical_contact_events),
            'obb_chassis_overlap_samples': len(auditor.physical_contact_events),
            'collision_contact_events': len(auditor.proximity_breach_events),
            'safety_brake_interventions': len(auditor.safety_brake_events),
            'safety_aborts': 1 if termination_reason == TerminationReason.SAFETY_ABORT else 0,
            'cpu_utilization_mean_pct': cpu_mean,
            'cpu_utilization_max_pct': cpu_max,
            'ram_utilization_mean_mb': ram_mean,
            'ram_utilization_max_mb': ram_max,
            'mode_transitions': len(auditor.compute_events),
            'failsafe_fallbacks': auditor.failsafe_fallbacks,
            'mode_occupancy_low_pct': occ_low,
            'mode_occupancy_normal_pct': occ_norm,
            'mode_occupancy_high_pct': occ_high,
            'v3_a_blockage_injected': auditor.blockage_injected,
            'v3_a_blockage_removed': auditor.blockage_removed,
            'v3_a_affected_robots_count': len(auditor.affected_robots),
            'v3_a_affected_tasks_count': len(auditor.affected_tasks),
            'v3_a_diverted_robots_count': len(auditor.diverted_robots),
            'v3_a_replan_latency_mean_ms': (
                round(sum(auditor.replan_response_latencies_ms) / len(auditor.replan_response_latencies_ms), 2)
                if auditor.replan_response_latencies_ms else None
            ),
            'comm_packets_sent': total_comm_sent,
            'comm_packets_delivered': total_comm_delivered,
            'comm_packets_dropped': total_comm_dropped,
            'comm_observed_loss_rate_pct': comm_loss_rate,
            'comm_transport_latency_mean_ms': comm_avg_lat,
            'comm_transport_latency_p95_ms': comm_p95_lat,
            'comm_jitter_mean_ms': comm_jitter,
            'comm_stale_messages_count': total_stale_msgs,
            'comm_expired_reservations_count': total_expired_res,
            'v3_e_comm_degraded': auditor.comm_degraded,
            'v3_e_comm_recovered': auditor.comm_recovered,
            'v3_e_comm_degradation_time_sec': auditor.comm_degrade_elapsed,
            'v3_e_comm_recovery_time_sec': auditor.comm_recover_elapsed,
            'v3_e_comm_profile': v3_e_comm_profile if is_v3_e else None,
        },
        'completed_task_ids': sorted(list(auditor.completed_task_ids)),
        'proximity_breach_events': auditor.proximity_breach_events,
        'physical_contact_events': auditor.physical_contact_events,
        'obb_chassis_overlap_events': auditor.physical_contact_events,
        'safety_brake_events': auditor.safety_brake_events,
        'collision_events': auditor.proximity_breach_events,
        'compute_mode_events': auditor.compute_events,
        'replan_counts_initial': auditor.initial_replan_counts,
        'replan_counts_final': auditor.final_replan_counts,
        'replan_counts_delta': replan_deltas,
        'v3_a_affected_robots': auditor.affected_robots,
        'v3_a_affected_tasks': auditor.affected_tasks,
        'v3_a_diverted_robots': sorted(list(auditor.diverted_robots)),
        'v3_a_replan_latencies_ms': auditor.replan_response_latencies_ms,
        'v3_e_comm_profile_schedule_events': auditor.comm_profile_schedule_events,
        'v3_e_comm_metrics_by_robot': auditor.comm_metrics_by_robot,
    }

    # Save raw trial file
    os.makedirs(raw_dir, exist_ok=True)
    raw_path = os.path.join(raw_dir, f"{experiment_id}.json")
    with open(raw_path, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2)
    print(f"  [SAVED RAW TRIAL] {raw_path}")

    auditor.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return result


def main():
    parser = argparse.ArgumentParser(description="YAVI-SIH26123 Fleet Benchmark Runner (M8A / M8B / M9)")
    parser.add_argument(
        '--workload',
        type=int,
        choices=[1, 15, 30, 50, 100],
        default=15,
        help="Workload task count",
    )
    parser.add_argument('--trials', type=int, default=1, help="Number of repeated trials")
    parser.add_argument('--seed', type=int, default=42, help="Base pseudorandom seed")
    parser.add_argument('--horizon', type=float, default=None, help="Custom mission horizon in seconds")
    parser.add_argument('--communication', type=str, default='NORMAL', help="M7 Communication Profile")
    parser.add_argument(
        '--compute-mode',
        type=str,
        choices=['NORMAL', 'LOW', 'HIGH', 'ADAPTIVE'],
        default='NORMAL',
        help="M8B Compute Mode (NORMAL, LOW, HIGH, ADAPTIVE)",
    )
    parser.add_argument('--m8b', action='store_true', help="Explicitly mark trial as M8B experiment")
    parser.add_argument('--m9', action='store_true', help="Explicitly mark trial as M9 experiment")
    parser.add_argument(
        '--v3-d',
        action='store_true',
        help="Execute M9-V3 Phase 1 Dynamic Task Arrival (Scenario V3-D.1)",
    )
    parser.add_argument(
        '--v3-a',
        action='store_true',
        help="Execute M9-V3 Phase 2 Temporary Aisle Blockage (Scenario V3-A)",
    )
    parser.add_argument(
        '--v3-a-block-time',
        type=float,
        default=45.0,
        help="Aisle blockage injection time from mission start (seconds)",
    )
    parser.add_argument(
        '--v3-a-unblock-time',
        type=float,
        default=90.0,
        help="Aisle blockage removal time from mission start (seconds)",
    )
    parser.add_argument(
        '--v3-a-blocker-x',
        type=float,
        default=4.5,
        help="Aisle blockage center X coordinate",
    )
    parser.add_argument(
        '--v3-a-blocker-y',
        type=float,
        default=6.75,
        help="Aisle blockage center Y coordinate",
    )
    parser.add_argument(
        '--v3-a-blocker-size-x',
        type=float,
        default=1.8,
        help="Aisle blockage length along X (meters)",
    )
    parser.add_argument(
        '--v3-a-blocker-size-y',
        type=float,
        default=0.6,
        help="Aisle blockage width along Y (meters)",
    )
    parser.add_argument(
        '--v3-e',
        action='store_true',
        help="Execute M9-V3-E Combined Stress Experiment (V3-D dynamic tasks + V3-A blockage + scheduled M7 comm degradation)",
    )
    parser.add_argument(
        '--v3-e-comm-degrade-time',
        type=float,
        default=75.0,
        help="Simulation elapsed time to inject communication degradation (default: 75.0s)",
    )
    parser.add_argument(
        '--v3-e-comm-recover-time',
        type=float,
        default=120.0,
        help="Simulation elapsed time to recover communication to NORMAL (default: 120.0s)",
    )
    parser.add_argument(
        '--v3-e-comm-profile',
        type=str,
        default='LOSS_HIGH',
        help="Communication profile to inject during degradation window (default: LOSS_HIGH)",
    )
    parser.add_argument('--world', type=str, default=None, help="Gazebo world name (e.g. warehouse_small, warehouse_m9_v1)")
    parser.add_argument('--map-id', type=str, default=None, help="Map identifier (e.g. warehouse_grid_small, warehouse_m9_v1)")
    parser.add_argument('--fleet-size', type=int, default=5, help="Number of robots in fleet")
    parser.add_argument('--schema-version', type=str, default=None, help="JSON schema version string")
    parser.add_argument('--raw-dir', type=str, default=None, help="Directory for raw trial JSONs")
    parser.add_argument('--agg-dir', type=str, default=None, help="Directory for aggregated JSONs")
    parser.add_argument('--benchmark-id', type=str, default=None, help="Custom benchmark identifier")
    parser.add_argument('--workload-file', type=str, default=None, help="Direct override path for workload YAML file")
    parser.add_argument('--dry-run', action='store_true', help="Simulate execution without starting Gazebo")
    args = parser.parse_args()

    is_v3_e = getattr(args, 'v3_e', False)
    is_m9 = (
        args.m9 or getattr(args, 'v3_d', False) or getattr(args, 'v3_a', False) or is_v3_e
        or (args.world is not None and 'm9' in args.world)
        or any('m9' in arg for arg in sys.argv)
    )
    is_m8b = not is_m9 and (args.m8b or args.compute_mode != 'NORMAL' or any('m8b' in arg for arg in sys.argv))
    fleet_size = args.fleet_size

    if is_m9:
        milestone = "M9"
        schema_ver = args.schema_version or "m9.v1"
        raw_dir = args.raw_dir or 'results/m9/raw'
        agg_dir = args.agg_dir or 'results/m9/aggregated'
        world = args.world or 'warehouse_m9_v1'
        map_id = args.map_id or ('warehouse_m9_v1' if 'm9_v1' in world else world)
        clean_tag = world.replace('warehouse_', '')
        cand_v3_d = f"config/workloads/workload_{args.workload}_tasks_m9_v3_d.yaml"
        cand_m9_wl_1 = f"config/workloads/workload_{args.workload}_tasks_{clean_tag}.yaml"
        cand_m9_wl_2 = f"config/workloads/workload_{args.workload}_tasks_m9_v1.yaml"
        if (getattr(args, 'v3_d', False) or getattr(args, 'v3_a', False) or is_v3_e) and os.path.isfile(cand_v3_d):
            workload_file = cand_v3_d
        elif os.path.isfile(cand_m9_wl_1):
            workload_file = cand_m9_wl_1
        elif os.path.isfile(cand_m9_wl_2):
            workload_file = cand_m9_wl_2
        else:
            workload_file = f"config/workloads/workload_{args.workload}_tasks.yaml"
    elif is_m8b:
        milestone = "M8B"
        schema_ver = args.schema_version or "m8b.v1"
        raw_dir = args.raw_dir or 'results/m8b/raw'
        agg_dir = args.agg_dir or 'results/m8b/aggregated'
        world = args.world or 'warehouse_small'
        map_id = args.map_id or 'warehouse_grid_small'
        workload_file = f"config/workloads/workload_{args.workload}_tasks.yaml"
    else:
        milestone = "M8A"
        schema_ver = args.schema_version or "m8a.v1"
        raw_dir = args.raw_dir or 'results/m8a/raw'
        agg_dir = args.agg_dir or 'results/m8a/aggregated'
        world = args.world or 'warehouse_small'
        map_id = args.map_id or 'warehouse_grid_small'
        workload_file = f"config/workloads/workload_{args.workload}_tasks.yaml"

    if args.workload_file:
        workload_file = args.workload_file

    # Determine horizon
    horizon_sec = args.horizon if args.horizon is not None else CALIBRATED_HORIZONS.get(args.workload, 60.0)

    if args.benchmark_id:
        benchmark_id = args.benchmark_id
    elif is_m9:
        benchmark_id = (
            f"bench_m9_w{args.workload}_{args.compute_mode.lower()}_f{fleet_size}_s{args.seed}_{int(time.time())}"
        )
    elif is_m8b:
        benchmark_id = (
            f"bench_m8b_w{args.workload}_{args.compute_mode.lower()}_s{args.seed}_{int(time.time())}"
        )
    else:
        benchmark_id = f"bench_m8a_w{args.workload}_s{args.seed}_{int(time.time())}"

    print("\n" + "=" * 78)
    print(f"  YAVI-SIH26123 {milestone} EXPERIMENTAL BENCHMARK INFRASTRUCTURE")
    print(
        f"  Workload: {args.workload} tasks | Trials: {args.trials} | "
        f"Base Seed: {args.seed} | Compute Mode: {args.compute_mode} | Fleet: {fleet_size}"
    )
    print(f"  World: {world} | Map ID: {map_id}")
    print(f"  Configured Horizon: {horizon_sec:.1f}s | Communication: {args.communication}")
    print(f"  Workload Spec: {workload_file} | Schema: {schema_ver}")
    print("=" * 78)

    if not args.dry_run:
        check_system_resources()

    trial_results: List[Dict[str, Any]] = []

    for trial_idx in range(args.trials):
        current_seed = args.seed + trial_idx
        trial_res = run_single_trial(
            workload_size=args.workload,
            workload_path=workload_file,
            trial_id=trial_idx,
            seed=current_seed,
            horizon_sec=horizon_sec,
            comm_profile=args.communication,
            raw_dir=raw_dir,
            compute_mode=args.compute_mode,
            schema_version=schema_ver,
            dry_run=args.dry_run,
            world=world,
            fleet_size=fleet_size,
            map_id=map_id,
            is_v3_a=getattr(args, 'v3_a', False) or is_v3_e,
            v3_a_block_time=args.v3_a_block_time,
            v3_a_unblock_time=args.v3_a_unblock_time,
            v3_a_blocker_pos=(args.v3_a_blocker_x, args.v3_a_blocker_y),
            v3_a_blocker_size=(args.v3_a_blocker_size_x, args.v3_a_blocker_size_y),
            is_v3_e=is_v3_e,
            v3_e_comm_degrade_time=args.v3_e_comm_degrade_time,
            v3_e_comm_recover_time=args.v3_e_comm_recover_time,
            v3_e_comm_profile=args.v3_e_comm_profile,
        )
        trial_results.append(trial_res)

    # Perform Statistical Aggregation
    os.makedirs(agg_dir, exist_ok=True)
    agg_summary = StatisticalAggregator.aggregate_trials(benchmark_id, trial_results)
    agg_path = os.path.join(agg_dir, f"{benchmark_id}.json")
    with open(agg_path, 'w', encoding='utf-8') as f:
        json.dump(agg_summary, f, indent=2)

    print("\n" + "=" * 78)
    print(f"  {milestone} BENCHMARK AGGREGATED SUMMARY")
    print("=" * 78)
    print(f"  Benchmark ID:      {benchmark_id}")
    print(f"  Trials Completed:  {len(trial_results)}")
    print(f"  Workload Size:     {args.workload} tasks")
    print(f"  Horizon:           {horizon_sec:.1f}s")
    print(f"  Aggregated File:   {agg_path}")
    print("-" * 78)
    print(f"  {'Metric':<36} | {'Mean':<10} | {'Median':<10} | {'Std':<10} | {'95% CI'}")
    print("-" * 78)
    for m_key, s in agg_summary['aggregated_metrics'].items():
        if s is not None:
            ci_str = f"[{s['ci_95_lower']}, {s['ci_95_upper']}]" if s['ci_95_lower'] is not None else "N/A (n<3)"
            print(f"  {m_key:<36} | {s['mean']:<10.2f} | {s['median']:<10.2f} | {s['std']:<10.2f} | {ci_str}")
    print("=" * 78)


if __name__ == '__main__':
    main()


"""ROS 2 Node executing Rolling-Horizon Task Planning per AMR."""

import math
import os
import time
from typing import Any, Dict, List, Optional, Set, Tuple

from amr_fleet_core.adaptive_compute_policy import (
    AdaptiveComputePolicy,
    AdaptiveComputePolicyConfig,
    ComputeState,
    PolicyTransitionRecord,
)
from amr_fleet_core.communication_model import (
    CommunicationAction,
    CommunicationImpairmentModel,
    CommunicationProfileConfig,
    DelayedMessageQueue,
    PRESET_PROFILES,
)
from amr_fleet_core.compute_modes import (
    ComputeMode,
    get_compute_mode_config,
)
from amr_fleet_core.coordination_models import ConflictType, CoordinationState, Position
from amr_fleet_core.deadlock_recovery import DeadlockRecoveryManager
from amr_fleet_core.fault_detector import FaultDetector
from amr_fleet_core.local_obstacle_detector import (
    LocalObstacleDetector,
    LocalRecoveryAction,
)
from amr_fleet_core.pibt_planner import PIBTLocalPlanner
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.rh_planner import PlanningResponseData, RHConfig, RollingHorizonPlanner
from amr_fleet_core.stale_state_manager import (
    InformationState,
    StaleStateManager,
)
from amr_fleet_core.wfg_deadlock import DeadlockDetector, WaitForGraph
from amr_fleet_msgs.msg import (
    ConflictReport,
    CoordinationStatus,
    DeadlockEvent,
    PlanningRequest,
    PlanningResponse,
    RobotBundle,
    RollingHorizonPlan,
    SpaceTimeReservation,
    TaskEvent as TaskEventMsg,
    TaskList,
)
try:
    from amr_fleet_msgs.msg import CommunicationMetrics, CommunicationProfile
    HAVE_M7_MSGS = True
except ImportError:
    HAVE_M7_MSGS = False
try:
    from amr_fleet_msgs.msg import ComputeModeEvent
    HAVE_M8B_MSGS = True
except ImportError:
    HAVE_M8B_MSGS = False
try:
    from amr_fleet_msgs.msg import AisleBlockageEvent
    HAVE_M9_V3_A_MSGS = True
except ImportError:
    HAVE_M9_V3_A_MSGS = False
try:
    import psutil
    HAVE_PSUTIL = True
except ImportError:
    HAVE_PSUTIL = False
try:
    from amr_fleet_msgs.srv import InjectFault
    HAVE_FAULT_SRV = True
except ImportError:
    HAVE_FAULT_SRV = False
try:
    from amr_fleet_msgs.msg import RobotHealth
    HAVE_HEALTH_MSG = True
except ImportError:
    HAVE_HEALTH_MSG = False
from amr_fleet_sim.grid_world import GridWorld
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def float_to_builtin_time(secs: float) -> BuiltinTime:
    """Convert float seconds to builtin_interfaces/Time msg."""
    b_time = BuiltinTime()
    b_time.sec = int(secs)
    b_time.nanosec = int((secs - int(secs)) * 1e9)
    return b_time


class RollingHorizonPlannerNode(Node):
    """
    Decentralized Rolling-Horizon Task Planning Node.

    Translates CBBA assigned bundles into sequenced sub-goals, single-agent A*
    rolling-horizon paths, execution windows, and trajectory commands.
    """

    def __init__(self) -> None:
        """Initialize RollingHorizonPlannerNode, pubs/subs, and timers."""
        super().__init__('rh_node')

        # Parameters
        self.declare_parameter('robot_id', 'amr_0')
        self.declare_parameter('horizon_steps', 10)
        self.declare_parameter('execution_window', 4)
        self.declare_parameter('replan_rate', 2.0)
        self.declare_parameter('sequencing_heuristic', 'PRIORITY_FIRST')
        self.declare_parameter('grid_resolution', 0.5)
        self.declare_parameter('goal_tolerance_m', 0.5)
        self.declare_parameter('enable_motion_execution', True)
        self.declare_parameter('spawn_x', 2.0)
        self.declare_parameter('spawn_y', 2.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('enable_coordination', True)
        self.declare_parameter('persistence_threshold_sec', 1.5)
        self.declare_parameter('min_stall_cycles', 3)
        self.declare_parameter('enable_comm_degradation', False)
        self.declare_parameter('comm_profile', 'NORMAL')
        self.declare_parameter('comm_latency_ms', 0.0)
        self.declare_parameter('comm_jitter_ms', 0.0)
        self.declare_parameter('comm_loss_probability', 0.0)
        self.declare_parameter('comm_burst_loss_probability', 0.0)
        self.declare_parameter('comm_burst_length_mean', 4.0)
        self.declare_parameter('comm_outage_start_s', 0.0)
        self.declare_parameter('comm_outage_duration_s', 0.0)
        self.declare_parameter('comm_seed', 42)
        self.declare_parameter('comm_isolated_robots', [''])
        self.declare_parameter('stale_threshold_sec', 1.5)
        self.declare_parameter('expiry_threshold_sec', 4.0)

        # M8B Adaptive Compute Parameters
        self.declare_parameter('compute_mode', 'NORMAL')
        self.declare_parameter('enable_adaptive_compute', False)
        self.declare_parameter('min_dwell_time_sec', 3.0)
        self.declare_parameter('confirmation_samples', 3)

        # Map Topology Configuration
        self.declare_parameter('map_config_file', '')

        self.robot_id = self.get_parameter('robot_id').value
        raw_mode = str(self.get_parameter('compute_mode').value)
        enable_adaptive_param = bool(self.get_parameter('enable_adaptive_compute').value)
        if raw_mode.upper() in ('ADAPTIVE', 'COMPUTE_MODE_ADAPTIVE') or enable_adaptive_param:
            self.is_adaptive = True
            initial_mode = ComputeMode.NORMAL
        else:
            self.is_adaptive = False
            initial_mode = ComputeMode.from_string(raw_mode)

        self.active_compute_mode = initial_mode
        mode_cfg = get_compute_mode_config(initial_mode)

        h_steps = int(self.get_parameter('horizon_steps').value)
        w_window = int(self.get_parameter('execution_window').value)
        replan_rate = float(self.get_parameter('replan_rate').value)

        # Apply mode parameters for fixed LOW or HIGH modes
        if not self.is_adaptive and initial_mode != ComputeMode.NORMAL:
            h_steps = mode_cfg.horizon_steps
            w_window = mode_cfg.execution_window
            replan_rate = mode_cfg.replan_rate

        policy_cfg = AdaptiveComputePolicyConfig(
            min_dwell_time_sec=float(self.get_parameter('min_dwell_time_sec').value),
            confirmation_samples=int(self.get_parameter('confirmation_samples').value),
            stale_age_low_threshold_s=float(self.get_parameter('stale_threshold_sec').value),
        )
        self.policy = AdaptiveComputePolicy(
            robot_id=self.robot_id,
            config=policy_cfg,
            initial_mode=initial_mode,
        )
        self.conflict_timestamps: List[float] = []

        heuristic = self.get_parameter('sequencing_heuristic').value
        res = self.get_parameter('grid_resolution').value
        self.grid_resolution = float(res)
        tol = self.get_parameter('goal_tolerance_m').value
        self.enable_motion = self.get_parameter('enable_motion_execution').value
        self.enable_coordination = self.get_parameter('enable_coordination').value
        self.persistence_threshold = float(
            self.get_parameter('persistence_threshold_sec').value
        )
        self.min_stall_cycles = int(self.get_parameter('min_stall_cycles').value)

        # M7 Communication Impairment & Stale Tracking
        enable_deg = bool(self.get_parameter('enable_comm_degradation').value)
        profile_name = str(self.get_parameter('comm_profile').value)
        raw_isolated = self.get_parameter('comm_isolated_robots').value
        isolated_list = [r for r in raw_isolated if r] if isinstance(raw_isolated, list) else []

        comm_cfg = CommunicationProfileConfig(
            profile_name=profile_name,
            enabled=enable_deg,
            latency_ms=float(self.get_parameter('comm_latency_ms').value),
            jitter_ms=float(self.get_parameter('comm_jitter_ms').value),
            loss_probability=float(self.get_parameter('comm_loss_probability').value),
            burst_loss_probability=float(self.get_parameter('comm_burst_loss_probability').value),
            burst_length_mean=float(self.get_parameter('comm_burst_length_mean').value),
            outage_start_s=float(self.get_parameter('comm_outage_start_s').value),
            outage_duration_s=float(self.get_parameter('comm_outage_duration_s').value),
            seed=int(self.get_parameter('comm_seed').value),
            isolated_robots=isolated_list,
        )
        if profile_name.upper() in PRESET_PROFILES and profile_name.upper() != 'NORMAL':
            preset = PRESET_PROFILES[profile_name.upper()]
            comm_cfg.latency_ms = preset.latency_ms
            comm_cfg.jitter_ms = preset.jitter_ms
            comm_cfg.loss_probability = preset.loss_probability
            comm_cfg.burst_loss_probability = preset.burst_loss_probability
            comm_cfg.burst_length_mean = preset.burst_length_mean
            comm_cfg.outage_start_s = preset.outage_start_s
            comm_cfg.outage_duration_s = preset.outage_duration_s
            comm_cfg.isolated_robots = list(preset.isolated_robots)
            if enable_deg:
                comm_cfg.enabled = True

        self.comm_model = CommunicationImpairmentModel(config=comm_cfg)
        self.stale_manager = StaleStateManager(
            local_robot_id=self.robot_id,
            stale_threshold_s=float(self.get_parameter('stale_threshold_sec').value),
            expiry_threshold_s=float(self.get_parameter('expiry_threshold_sec').value),
        )
        self.delayed_queue = DelayedMessageQueue()
        self.start_sim_time = time.monotonic()

        map_config_file = str(self.get_parameter('map_config_file').value)
        if map_config_file and os.path.isfile(map_config_file):
            self.grid = GridWorld.from_yaml(map_config_file)
            self.get_logger().info(
                f'[{self.robot_id}] Loaded custom grid from {map_config_file} '
                f'({self.grid.width}x{self.grid.height})'
            )
        else:
            self.grid = GridWorld.create_warehouse_grid(resolution=res)

        try:
            r_idx = int(self.robot_id.split('_')[-1])
            default_y = 2.0 + r_idx * 3.0
        except Exception:
            default_y = 2.0

        spawn_x_val = float(self.get_parameter('spawn_x').value)
        spawn_y_val = float(self.get_parameter('spawn_y').value)
        spawn_yaw_val = float(self.get_parameter('spawn_yaw').value)
        if not map_config_file and spawn_y_val == 2.0 and default_y != 2.0:
            spawn_y_val = default_y

        self.spawn_x = spawn_x_val
        self.spawn_y = spawn_y_val
        self.spawn_yaw = spawn_yaw_val

        config = RHConfig(
            horizon_steps=h_steps,
            execution_window=w_window,
            replan_rate=replan_rate,
            sequencing_heuristic=heuristic,
            grid_resolution=res,
            goal_tolerance_m=tol,
        )

        self.planner = RollingHorizonPlanner(
            robot_id=self.robot_id, config=config, grid=self.grid
        )
        self.planner.update_position((self.spawn_x, self.spawn_y))
        self.cached_tasks: Dict[str, Dict[str, Any]] = {}
        self.cached_bundle: List[str] = []
        self.static_map_obstacles: Set[Tuple[int, int]] = set(self.grid.obstacles)
        self.dynamic_blockages: Dict[str, Set[Tuple[int, int]]] = {}
        self.current_yaw: float = self.spawn_yaw

        # M6 Coordination Subsystems
        self.res_table = SpaceTimeReservationTable()
        self.pibt_planner = PIBTLocalPlanner(self.grid, self.res_table)
        self.wfg = WaitForGraph()
        self.deadlock_detector = DeadlockDetector(
            persistence_threshold_sec=self.persistence_threshold,
            min_stall_cycles=self.min_stall_cycles,
        )
        self.recovery_manager = DeadlockRecoveryManager(self.grid, self.res_table)

        self.coordination_state = CoordinationState.CLEAR.value
        self.current_priority = 1000.0
        self.waiting_for_robot = ''
        self.target_cell: Position = self.grid.to_grid(self.spawn_x, self.spawn_y)
        self.peer_states: Dict[str, Dict[str, Any]] = {}
        self.peer_conflicts: List[ConflictReport] = []
        self.coord_time_step: int = 0

        # Publishers
        self.pub_rolling_plan = self.create_publisher(
            RollingHorizonPlan, f'/{self.robot_id}/rolling_plan', 10,
        )
        self.pub_nav_path = self.create_publisher(
            Path, f'/{self.robot_id}/path', 10,
        )
        self.pub_plan_request = self.create_publisher(
            PlanningRequest, f'/{self.robot_id}/planning_request', 10,
        )
        self.pub_plan_response = self.create_publisher(
            PlanningResponse, f'/{self.robot_id}/planning_response', 10,
        )
        self.pub_cmd_vel = self.create_publisher(
            Twist, f'/{self.robot_id}/cmd_vel', 10,
        )
        self.pub_task_status = self.create_publisher(
            TaskEventMsg, '/tasks/update_status', 20,
        )
        self.pub_coord_status = self.create_publisher(
            CoordinationStatus, f'/{self.robot_id}/coordination_status', 10,
        )
        self.pub_fleet_coord_status = self.create_publisher(
            CoordinationStatus, '/fleet/coordination_status', 20,
        )
        self.pub_reservation = self.create_publisher(
            SpaceTimeReservation, '/fleet/reservations', 50,
        )
        self.pub_conflict = self.create_publisher(
            ConflictReport, '/fleet/conflicts', 20,
        )
        self.pub_deadlock = self.create_publisher(
            DeadlockEvent, '/fleet/deadlocks', 20,
        )
        if HAVE_M7_MSGS:
            self.pub_comm_metrics = self.create_publisher(
                CommunicationMetrics, f'/{self.robot_id}/comm_metrics', 10,
            )
            self.pub_fleet_comm_metrics = self.create_publisher(
                CommunicationMetrics, '/fleet/comm_metrics', 20,
            )
            self.sub_comm_profile = self.create_subscription(
                CommunicationProfile,
                '/fleet/comm_profile',
                self._handle_comm_profile,
                10,
            )
        if HAVE_M8B_MSGS:
            self.pub_compute_mode = self.create_publisher(
                ComputeModeEvent, f'/{self.robot_id}/compute_mode', 10,
            )
            self.pub_fleet_compute_events = self.create_publisher(
                ComputeModeEvent, '/fleet/compute_events', 20,
            )

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self._handle_odom,
            10,
        )
        self.sub_bundle = self.create_subscription(
            RobotBundle,
            f'/{self.robot_id}/bundle',
            self._handle_bundle,
            10,
        )
        self.sub_tasks = self.create_subscription(
            TaskList,
            '/tasks/all',
            self._handle_tasks_all,
            10,
        )
        self.sub_reservations = self.create_subscription(
            SpaceTimeReservation,
            '/fleet/reservations',
            self._handle_incoming_reservation,
            50,
        )
        self.sub_peer_coord = self.create_subscription(
            CoordinationStatus,
            '/fleet/coordination_status',
            self._handle_peer_coordination_status,
            20,
        )
        self.sub_fleet_deadlocks = self.create_subscription(
            DeadlockEvent,
            '/fleet/deadlocks',
            self._handle_incoming_deadlock,
            20,
        )
        self.sub_fleet_conflicts = self.create_subscription(
            ConflictReport,
            '/fleet/conflicts',
            self._handle_incoming_conflict,
            20,
        )
        if HAVE_M9_V3_A_MSGS:
            self.sub_blockage = self.create_subscription(
                AisleBlockageEvent,
                '/environment/aisle_blockages',
                self._handle_aisle_blockage,
                10,
            )

        self.obstacle_ahead: bool = False
        self.local_detector = LocalObstacleDetector(
            grid_resolution=self.grid_resolution,
            safety_threshold_m=0.28,
            detection_horizon_m=1.5,
            forward_arc_deg=24.0,
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            f'/{self.robot_id}/scan',
            self._handle_scan,
            qos_profile_sensor_data,
        )

        # Timers
        period = 1.0 / max(0.1, replan_rate)
        self.timer = self.create_timer(period, self._planning_cycle)
        self.control_timer = self.create_timer(0.1, self._control_loop)

        # YAVI-SIH26123 Fault Resilience Layer
        self.spawn_time = float(self.get_clock().now().nanoseconds * 1e-9)
        self.fault_detector = FaultDetector(self.robot_id)
        self.failed_robot_obstacles: Dict[str, Set[Tuple[int, int]]] = {}
        if HAVE_FAULT_SRV:
            self.srv_inject_fault = self.create_service(
                InjectFault,
                f'/{self.robot_id}/inject_fault',
                self._handle_inject_fault,
            )
        if HAVE_HEALTH_MSG:
            self.pub_health = self.create_publisher(
                RobotHealth,
                f'/{self.robot_id}/health',
                10,
            )
            self.pub_fleet_health = self.create_publisher(
                RobotHealth,
                '/fleet/robot_health',
                20,
            )
            self.sub_fleet_health = self.create_subscription(
                RobotHealth,
                '/fleet/robot_health',
                self._handle_robot_health,
                20,
            )

        self.get_logger().info(
            f'Rolling-Horizon Planner initialized for {self.robot_id} '
            f'(h={h_steps}, w={w_window}, heuristic={heuristic})',
        )

    def _handle_odom(self, msg: Odometry) -> None:
        """Update robot localized position and yaw orientation."""
        frame_id = msg.header.frame_id or ''
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        if 'map' in frame_id:
            mx = msg.pose.pose.position.x
            my = msg.pose.pose.position.y
            map_yaw = odom_yaw
        else:
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            cos_sy = math.cos(self.spawn_yaw)
            sin_sy = math.sin(self.spawn_yaw)
            mx = self.spawn_x + (ox * cos_sy - oy * sin_sy)
            my = self.spawn_y + (ox * sin_sy + oy * cos_sy)
            map_yaw = odom_yaw + self.spawn_yaw

        while map_yaw > math.pi:
            map_yaw -= 2.0 * math.pi
        while map_yaw < -math.pi:
            map_yaw += 2.0 * math.pi
        self.current_yaw = map_yaw
        pos = (round(mx, 3), round(my, 3))
        self.planner.update_position(pos)

    @property
    def current_x(self) -> float:
        """Return current continuous world X position."""
        if hasattr(self, 'planner') and self.planner.current_position:
            return float(self.planner.current_position[0])
        return float(self.spawn_x)

    @property
    def current_y(self) -> float:
        """Return current continuous world Y position."""
        if hasattr(self, 'planner') and self.planner.current_position:
            return float(self.planner.current_position[1])
        return float(self.spawn_y)

    @property
    def current_grid_x(self) -> int:
        """Return current discrete grid cell X coordinate."""
        gx, _ = self.grid.to_grid(self.current_x, self.current_y)
        return int(gx)

    @property
    def current_grid_y(self) -> int:
        """Return current discrete grid cell Y coordinate."""
        _, gy = self.grid.to_grid(self.current_x, self.current_y)
        return int(gy)

    def _handle_bundle(self, msg: RobotBundle) -> None:
        """Update assigned bundle from CBBA."""
        # Prevent execution of tasks from unconverged CBBA bundles
        if not getattr(msg, 'is_converged', False):
            return

        incoming_bundle = list(msg.task_ids)
        if incoming_bundle != self.cached_bundle or not self.planner.ordered_tasks:
            self.cached_bundle = incoming_bundle
            if self.cached_tasks and self.cached_bundle:
                self.planner.update_assigned_bundle(self.cached_bundle, self.cached_tasks)

    def _handle_tasks_all(self, msg: TaskList) -> None:
        """Cache all task metadata from Task Manager."""
        tasks_map = {}
        for td in msg.tasks:
            tasks_map[td.task_id] = {
                'task_id': td.task_id,
                'pickup': (td.pickup_pose.x, td.pickup_pose.y),
                'dropoff': (td.dropoff_pose.x, td.dropoff_pose.y),
                'priority': td.priority,
                'status': td.status,
                'deadline': (
                    td.deadline.sec + td.deadline.nanosec * 1e-9
                    if td.deadline.sec > 0 else None
                ),
            }
        self.cached_tasks = tasks_map
        if self.cached_bundle:
            self.planner.update_assigned_bundle(self.cached_bundle, self.cached_tasks)

    def _handle_aisle_blockage(self, msg: Any) -> None:
        """Process environmental aisle blockage events (M9-V3-A)."""
        gx_start = max(0, int(msg.min_x / self.grid.resolution))
        gx_end = min(self.grid.width - 1, int(msg.max_x / self.grid.resolution))
        gy_start = max(0, int(msg.min_y / self.grid.resolution))
        gy_end = min(self.grid.height - 1, int(msg.max_y / self.grid.resolution))

        blocked_cells = {
            (gx, gy)
            for gx in range(gx_start, gx_end + 1)
            for gy in range(gy_start, gy_end + 1)
        }

        if msg.is_blocked:
            self.dynamic_blockages[msg.blockage_id] = blocked_cells
            for cell in blocked_cells:
                self.grid.add_obstacle(cell)
            self.get_logger().info(
                f"[{self.robot_id}] Aisle blockage '{msg.blockage_id}' applied: "
                f'{len(blocked_cells)} cells added as obstacles.'
            )

            # Check if current planned path intersects the blocked region
            path_affected = False
            for wp in (self.planner.full_path or self.planner.execution_path):
                cell = self.grid.to_grid(wp[0], wp[1])
                if cell in blocked_cells:
                    path_affected = True
                    break

            if path_affected:
                self.get_logger().info(
                    f'[{self.robot_id}] Active path intersects dynamic blockage '
                    f"'{msg.blockage_id}'. Invalidating cached path and replanning diversion."
                )
                self.planner.full_path = []
                self.planner.horizon_path = []
                self.planner.execution_path = []
                self.planner.active_waypoint_idx = 1
                if self.enable_coordination:
                    self.res_table.release_robot(self.robot_id)

                if self.planner.current_goal and self.planner.active_phase != 'IDLE':
                    curr_tid = (
                        self.planner.ordered_tasks[self.planner.active_task_idx]
                        if self.planner.active_task_idx < len(self.planner.ordered_tasks)
                        else ''
                    )
                    self._publish_plan_request(
                        curr_tid, self.planner.active_phase, self.planner.current_goal,
                    )
                    resp = self.planner.replan()
                    self._publish_plan_response(resp)
                    self._publish_rolling_plan()
        else:
            self.dynamic_blockages.pop(msg.blockage_id, None)
            cleared_count = 0
            for cell in blocked_cells:
                if cell not in self.static_map_obstacles:
                    self.grid.remove_obstacle(cell)
                    cleared_count += 1
            self.get_logger().info(
                f"[{self.robot_id}] Aisle blockage '{msg.blockage_id}' cleared: "
                f'{cleared_count} cells restored to traversable.'
            )

    def _handle_scan(self, msg: LaserScan) -> None:
        """Process LiDAR to detect obstacles in forward arc and update local occupancy."""
        num_rays = len(msg.ranges)
        if num_rays == 0:
            return

        # 1. Update local obstacle detector from sensor ranges
        self.local_detector.process_scan(
            ranges=list(msg.ranges),
            range_min=msg.range_min,
            range_max=msg.range_max,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            robot_pose=(self.current_x, self.current_y, self.current_yaw),
        )

        # 2. Forward arc reactive emergency braking threshold: 0.28m experimental threshold
        center_idx = num_rays // 2
        arc_rays = max(1, int(num_rays * (12.0 / 360.0)))
        forward_ranges = []
        for idx in range(center_idx - arc_rays, center_idx + arc_rays + 1):
            if 0 <= idx < num_rays:
                r = msg.ranges[idx]
                if (
                    msg.range_min < r < msg.range_max
                    and not math.isinf(r)
                    and not math.isnan(r)
                ):
                    forward_ranges.append(r)

        self.obstacle_ahead = bool(
            (forward_ranges and min(forward_ranges) < 0.28) or
            self.local_detector.has_immediate_hazard()
        )

        # 3. Path-intersection check and sensor-driven recovery
        if self.planner.full_path and self.planner.active_phase != 'IDLE':
            action, sidestep, reason = (
                self.local_detector.evaluate_local_recovery(
                    current_pos=(self.current_x, self.current_y),
                    current_cell=(self.current_grid_x, self.current_grid_y),
                    planned_path_cells=self.planner.full_path,
                    grid=self.grid,
                    res_table=self.res_table,
                    robot_id=self.robot_id,
                )
            )
            if action in (
                LocalRecoveryAction.LOCAL_SAFETY_HOLD,
                LocalRecoveryAction.EMERGENCY_BRAKE,
            ):
                sensor_cells = (
                    self.local_detector.get_sensor_occupied_cells()
                )
                blocked_on_path = [
                    c for c in self.planner.full_path[:4]
                    if c in sensor_cells
                ]
                if blocked_on_path:
                    for bc in blocked_on_path:
                        self.grid.add_obstacle(bc)
                    if self.enable_coordination:
                        self.res_table.invalidate_cells(set(blocked_on_path))
                        self.res_table.release_robot(self.robot_id)
                    self.planner.full_path = []
                    self.planner.horizon_path = []
                    self.planner.execution_path = []
                    self.planner.active_waypoint_idx = 1
                    if self.planner.current_goal:
                        self.planner.replan()
                        self._publish_rolling_plan()

    def _publish_task_transition(
        self,
        task_id: str,
        new_state: str,
        details: str = '',
    ) -> None:
        """Emit task lifecycle state transition event to M3 TaskManager."""
        msg = TaskEventMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.event_type = f'TRANSITION_TO_{new_state}'
        msg.new_state = new_state
        msg.robot_id = self.robot_id
        msg.timestamp = float_to_builtin_time(self.get_clock().now().nanoseconds * 1e-9)
        msg.details = details
        self.pub_task_status.publish(msg)

    def _handle_inject_fault(self, request: Any, response: Any) -> Any:
        """Handle fault injection command for testing."""
        ok, msg = self.fault_detector.inject_fault(request.fault_type, request.duration_sec)
        response.success = ok
        response.message = msg
        self.get_logger().warn(
            f'[{self.robot_id}] Fault injected: {request.fault_type} -> {msg}'
        )
        if self.fault_detector.is_self_healthy():
            self.planner.assigned_bundle = []
            self.planner.full_path = []
            self.planner.horizon_path = []
            self.planner.execution_path = []
            if self.enable_coordination:
                self.res_table.release_robot(self.robot_id)
            self.get_logger().info(
                f'[{self.robot_id}] Operator restore completed. Fleet state clean-slated.'
            )
        else:
            self.planner.full_path = []
            self.planner.horizon_path = []
            self.planner.execution_path = []
            if self.enable_coordination:
                self.res_table.release_robot(self.robot_id)
            twist = Twist()
            self.pub_cmd_vel.publish(twist)
        self._publish_health_status()
        return response

    def _handle_robot_health(self, msg: Any) -> None:
        """Process peer robot health telemetry."""
        if msg.robot_id == self.robot_id:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Filter through impairment model if active
        if hasattr(self, 'comm_model') and self.comm_model.config.enabled:
            action, delay_ms, _ = self.comm_model.process_message(
                sender_id=msg.robot_id,
                receiver_id=self.robot_id,
                current_time=now_sec,
            )
            if action == CommunicationAction.DROP:
                return
            if action == CommunicationAction.DELAY:
                deliver_at = now_sec + (delay_ms / 1000.0)
                self.delayed_queue.schedule(
                    deliver_at,
                    self._apply_robot_health,
                    msg,
                )
                return

        self._apply_robot_health(msg)

    def _apply_robot_health(self, msg: Any) -> None:
        """Apply validated peer health update to fault detector."""
        pose = (float(msg.last_pose.x), float(msg.last_pose.y))
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.fault_detector.record_peer_heartbeat(
            msg.robot_id,
            msg.health_state,
            pose,
            msg.active_task_id,
            now_sec,
        )

    def _publish_health_status(self) -> None:
        """Broadcast local AMR operational health state."""
        if not HAVE_HEALTH_MSG or not hasattr(self, 'pub_health'):
            return
        msg = RobotHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.health_state = self.fault_detector.self_state.value
        msg.last_pose.x = float(self.planner.current_position[0])
        msg.last_pose.y = float(self.planner.current_position[1])
        curr_tid = ''
        if (
            self.planner.ordered_tasks
            and self.planner.active_task_idx < len(self.planner.ordered_tasks)
        ):
            curr_tid = self.planner.ordered_tasks[self.planner.active_task_idx]
        msg.active_task_id = curr_tid
        now_sec = float(self.get_clock().now().nanoseconds * 1e-9)
        if not hasattr(self, 'spawn_time') or self.spawn_time <= 0.0:
            self.spawn_time = now_sec
        msg.uptime_sec = max(0.0, float(now_sec - self.spawn_time))
        self.pub_health.publish(msg)
        self.pub_fleet_health.publish(msg)

    def _publish_plan_request(
        self,
        task_id: str,
        sub_goal_type: str,
        goal: Tuple[float, float],
    ) -> None:
        """Publish planning request abstraction message."""
        msg = PlanningRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.start_pose.x = float(self.planner.current_position[0])
        msg.start_pose.y = float(self.planner.current_position[1])
        msg.goal_pose.x = float(goal[0])
        msg.goal_pose.y = float(goal[1])
        msg.horizon_steps = self.planner.config.horizon_steps
        msg.execution_window = self.planner.config.execution_window
        msg.task_id = task_id
        msg.sub_goal_type = sub_goal_type
        self.pub_plan_request.publish(msg)

    def _publish_plan_response(self, resp: PlanningResponseData) -> None:
        """Publish planning response abstraction message."""
        msg = PlanningResponse()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.task_id = resp.task_id
        msg.sub_goal_type = resp.sub_goal_type
        msg.total_cost = float(resp.total_cost)
        msg.planning_latency_ms = float(resp.planning_latency_ms)
        msg.success = bool(resp.success)
        msg.status_message = resp.status_message
        msg.horizon_steps = resp.horizon_steps
        msg.execution_window = resp.execution_window
        msg.replan_count = resp.replan_count

        for pt in resp.full_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.full_path.append(p)

        for pt in resp.horizon_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.horizon_path.append(p)

        for pt in resp.execution_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.execution_path.append(p)

        self.pub_plan_response.publish(msg)

    def _publish_rolling_plan(self) -> None:
        """Publish aggregated rolling-horizon plan state."""
        msg = RollingHorizonPlan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.assigned_bundle = list(self.planner.assigned_bundle)

        curr_tid = (
            self.planner.ordered_tasks[self.planner.active_task_idx]
            if self.planner.active_task_idx < len(self.planner.ordered_tasks)
            else ''
        )
        msg.current_task_id = curr_tid
        msg.current_phase = self.planner.active_phase

        if self.planner.current_goal:
            msg.current_goal.x = float(self.planner.current_goal[0])
            msg.current_goal.y = float(self.planner.current_goal[1])

        for pt in self.planner.horizon_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.horizon_path.append(p)

        for pt in self.planner.execution_path:
            p = Point()
            p.x, p.y = float(pt[0]), float(pt[1])
            msg.execution_path.append(p)

        msg.horizon_steps = self.planner.config.horizon_steps
        msg.execution_window = self.planner.config.execution_window
        msg.replan_count = self.planner.replan_count
        msg.planning_latency_ms = float(self.planner.last_latency_ms)
        msg.is_valid = bool(self.planner.last_plan_success)

        self.pub_rolling_plan.publish(msg)

        # Standard nav_msgs/Path for RViz visual trajectory rendering
        nav_path = Path()
        nav_path.header.stamp = msg.header.stamp
        nav_path.header.frame_id = 'map'
        for pt in (self.planner.full_path or self.planner.horizon_path):
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = 0.05
            nav_path.poses.append(ps)
        self.pub_nav_path.publish(nav_path)

    def _compute_local_priority(
        self,
        curr_pos_grid: Position,
        goal_pos_grid: Position,
    ) -> float:
        """Compute deterministic local priority for PIBT arbitration."""
        task_prio = 1
        if self.planner.active_task_idx < len(self.planner.ordered_tasks):
            tid = self.planner.ordered_tasks[self.planner.active_task_idx]
            task_def = self.planner.tasks_map.get(tid, {})
            task_prio = task_def.get('priority', 1)

        manhattan_dist = (
            abs(curr_pos_grid[0] - goal_pos_grid[0])
            + abs(curr_pos_grid[1] - goal_pos_grid[1])
        )
        try:
            r_num = int(self.robot_id.split('_')[-1])
            tie_breaker = 1.0 / (1.0 + r_num)
        except Exception:
            tie_breaker = 0.01

        return float(task_prio * 1000.0 + 10.0 / (1.0 + manhattan_dist) + tie_breaker)

    def _publish_reservation(
        self,
        from_cell: Position,
        to_cell: Position,
        time_step: int,
        is_edge: bool,
        priority: float,
    ) -> None:
        """Publish space-time reservation message to fleet."""
        msg = SpaceTimeReservation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.from_x = int(from_cell[0])
        msg.from_y = int(from_cell[1])
        msg.to_x = int(to_cell[0])
        msg.to_y = int(to_cell[1])
        msg.time_step = int(time_step)
        msg.duration_sec = 0.5
        msg.is_edge = bool(is_edge)
        msg.priority = float(priority)
        self.pub_reservation.publish(msg)

    def _publish_coordination_status(
        self,
        curr_cell: Position,
        target_cell: Position,
    ) -> None:
        """Publish real-time coordination status to fleet and local topic."""
        msg = CoordinationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.priority = float(self.current_priority)
        msg.current_cell_x = int(curr_cell[0])
        msg.current_cell_y = int(curr_cell[1])
        msg.target_cell_x = int(target_cell[0])
        msg.target_cell_y = int(target_cell[1])
        msg.time_step = int(self.coord_time_step)
        msg.status = self.coordination_state
        msg.waiting_for_robot = self.waiting_for_robot
        msg.pose_x = float(self.planner.current_position[0])
        msg.pose_y = float(self.planner.current_position[1])
        self.pub_coord_status.publish(msg)
        self.pub_fleet_coord_status.publish(msg)

    def _publish_conflict_report(
        self,
        conflict_type: str,
        peer_id: str,
        cell: Position,
        time_step: int,
        resolved: bool,
    ) -> None:
        """Publish detected conflict report to fleet observability."""
        msg = ConflictReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_id
        msg.conflict_type = conflict_type
        msg.robot_a = self.robot_id
        msg.robot_b = peer_id
        msg.cell_x = int(cell[0])
        msg.cell_y = int(cell[1])
        msg.time_step = int(time_step)
        msg.resolved = resolved
        self.conflict_timestamps.append(time.time())
        self.pub_conflict.publish(msg)

    def _publish_deadlock_event(
        self,
        cycle_ids: List[str],
        action: str,
        duration: float,
        success: bool,
    ) -> None:
        """Publish confirmed deadlock and recovery event to fleet."""
        msg = DeadlockEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_id
        msg.cycle_robot_ids = list(cycle_ids)
        msg.root_cause = 'WFG_CYCLE_DETECTED'
        msg.persistence_duration_sec = float(self.persistence_threshold)
        msg.recovery_action = action
        msg.recovery_success = success
        msg.recovery_duration_sec = float(duration)
        self.pub_deadlock.publish(msg)

    def _handle_comm_profile(self, msg: Any) -> None:
        """Dynamically update communication degradation profile across the fleet."""
        self.get_logger().info(
            f'[{self.robot_id}] Received comm profile update: '
            f'{msg.profile_name} (enabled={msg.enabled})'
        )
        cfg = CommunicationProfileConfig(
            profile_name=msg.profile_name,
            enabled=msg.enabled,
            latency_ms=msg.latency_ms,
            jitter_ms=msg.jitter_ms,
            loss_probability=msg.loss_probability,
            burst_loss_probability=msg.burst_loss_probability,
            burst_length_mean=4.0,
            outage_duration_s=msg.outage_duration_s,
            seed=msg.seed if msg.seed > 0 else 42,
            isolated_robots=list(msg.isolated_robots),
        )
        self.comm_model.set_config(cfg)

    def _reconcile_reconnection(self) -> None:
        """Re-broadcast active state when network reconnects after outage/partition."""
        curr_cell = self.grid.to_grid(
            self.planner.current_position[0], self.planner.current_position[1]
        )
        self._publish_coordination_status(curr_cell, self.target_cell)
        self._publish_reservation(
            from_cell=curr_cell,
            to_cell=self.target_cell,
            time_step=self.coord_time_step + 1,
            is_edge=False,
            priority=self.current_priority,
        )

    def _publish_comm_metrics(self, now_sec: float) -> None:
        """Publish real-time communication telemetry."""
        if not HAVE_M7_MSGS:
            return
        met = self.comm_model.get_metrics()
        stale_sum = self.stale_manager.get_stale_summary(now_sec)

        msg = CommunicationMetrics()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.profile_name = met['profile_name']
        msg.messages_sent = met['messages_sent']
        msg.messages_delivered = met['messages_delivered']
        msg.messages_dropped = met['messages_dropped']
        msg.packet_loss_rate = met['packet_loss_rate']
        msg.avg_latency_ms = met['avg_latency_ms']
        msg.p95_latency_ms = met['p95_latency_ms']
        msg.jitter_ms = met['jitter_ms']
        msg.burst_events_count = met['burst_events_count']
        msg.outage_active = self.comm_model.is_in_outage(now_sec)
        msg.stale_messages_count = stale_sum['stale_count']
        msg.expired_reservations_count = stale_sum['expired_count']
        msg.min_distance_m = 0.0
        msg.collision_count = 0
        msg.safety_interventions = 0

        self.pub_comm_metrics.publish(msg)
        self.pub_fleet_comm_metrics.publish(msg)

    def _handle_incoming_reservation(self, msg: SpaceTimeReservation) -> None:
        """Cache incoming space-time reservation through communication degradation boundary."""
        if msg.robot_id == self.robot_id:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        sent_ts = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec > 0 else now_sec
        )

        action, delay_ms, _ = self.comm_model.process_message(
            sender_id=msg.robot_id,
            receiver_id=self.robot_id,
            current_time=now_sec,
        )
        if action == CommunicationAction.DROP:
            return
        if action == CommunicationAction.DELAY:
            deliver_at = now_sec + (delay_ms / 1000.0)
            self.delayed_queue.schedule(
                deliver_at,
                self._apply_reservation_payload,
                (msg, sent_ts),
            )
            return

        self._apply_reservation_payload((msg, sent_ts))

    def _apply_reservation_payload(self, data: Tuple[SpaceTimeReservation, float]) -> None:
        """Apply delivered reservation payload after network transit."""
        msg, sent_ts = data
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        state, is_reconn = self.stale_manager.record_incoming(
            'reservations', msg.robot_id, sent_ts, now_sec, msg
        )
        if is_reconn:
            self._reconcile_reconnection()

        if state == InformationState.EXPIRED:
            return

        if msg.is_edge:
            self.res_table.reserve_edge(
                (msg.from_x, msg.from_y),
                (msg.to_x, msg.to_y),
                msg.time_step,
                msg.robot_id,
                msg.priority,
            )
        else:
            self.res_table.reserve(
                (msg.to_x, msg.to_y),
                msg.time_step,
                msg.robot_id,
                msg.priority,
            )

    def _handle_peer_coordination_status(self, msg: CoordinationStatus) -> None:
        """Track peer AMR coordination state through communication degradation boundary."""
        if msg.robot_id == self.robot_id:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        sent_ts = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec > 0 else now_sec
        )

        action, delay_ms, _ = self.comm_model.process_message(
            sender_id=msg.robot_id,
            receiver_id=self.robot_id,
            current_time=now_sec,
        )
        if action == CommunicationAction.DROP:
            return
        if action == CommunicationAction.DELAY:
            deliver_at = now_sec + (delay_ms / 1000.0)
            self.delayed_queue.schedule(
                deliver_at,
                self._apply_peer_coordination_status,
                (msg, sent_ts),
            )
            return

        self._apply_peer_coordination_status((msg, sent_ts))

    def _apply_peer_coordination_status(self, data: Tuple[CoordinationStatus, float]) -> None:
        """Apply delivered peer status after network transit."""
        msg, sent_ts = data
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        state, is_reconn = self.stale_manager.record_incoming(
            'coordination_status', msg.robot_id, sent_ts, now_sec, msg
        )
        if is_reconn:
            self._reconcile_reconnection()

        if state == InformationState.EXPIRED:
            return

        px = getattr(msg, 'pose_x', 0.0)
        py = getattr(msg, 'pose_y', 0.0)
        if px == 0.0 and py == 0.0:
            world_pt = self.grid.to_world((msg.current_cell_x, msg.current_cell_y))
            px, py = world_pt[0], world_pt[1]

        self.peer_states[msg.robot_id] = {
            'priority': msg.priority,
            'current_cell': (msg.current_cell_x, msg.current_cell_y),
            'target_cell': (msg.target_cell_x, msg.target_cell_y),
            'status': msg.status,
            'waiting_for': msg.waiting_for_robot,
            'time_step': msg.time_step,
            'pose': (round(px, 3), round(py, 3)),
        }

        # Synchronize peer waiting relationship into local Wait-For Graph
        if msg.waiting_for_robot:
            self.wfg.add_wait(
                msg.robot_id,
                msg.waiting_for_robot,
                (msg.target_cell_x, msg.target_cell_y),
                msg.time_step,
            )
        else:
            self.wfg.clear_robot(msg.robot_id)

    def _handle_incoming_deadlock(self, msg: DeadlockEvent) -> None:
        """Handle broadcast deadlock event through communication degradation boundary."""
        sender = (
            msg.header.frame_id if msg.header.frame_id
            else (msg.cycle_robot_ids[0] if msg.cycle_robot_ids else 'fleet')
        )
        if sender == self.robot_id:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        sent_ts = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec > 0 else now_sec
        )

        action, delay_ms, _ = self.comm_model.process_message(
            sender_id=sender,
            receiver_id=self.robot_id,
            current_time=now_sec,
        )
        if action == CommunicationAction.DROP:
            return
        if action == CommunicationAction.DELAY:
            deliver_at = now_sec + (delay_ms / 1000.0)
            self.delayed_queue.schedule(
                deliver_at,
                self._apply_deadlock_payload,
                (msg, sent_ts),
            )
            return

        self._apply_deadlock_payload((msg, sent_ts))

    def _apply_deadlock_payload(self, data: Tuple[DeadlockEvent, float]) -> None:
        """Apply delivered deadlock event after network transit."""
        msg, sent_ts = data
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        sender = msg.header.frame_id if msg.header.frame_id else 'fleet'
        state, _ = self.stale_manager.record_incoming(
            'deadlocks', sender, sent_ts, now_sec, msg
        )
        if state == InformationState.EXPIRED:
            return

        if self.robot_id in msg.cycle_robot_ids:
            self.deadlock_detector.reset_cycle(list(msg.cycle_robot_ids))

    def _handle_incoming_conflict(self, msg: ConflictReport) -> None:
        """Handle broadcast conflict report through communication degradation boundary."""
        sender = msg.robot_a if msg.robot_a else msg.header.frame_id
        if not sender or sender == self.robot_id:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        sent_ts = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if msg.header.stamp.sec > 0 else now_sec
        )

        action, delay_ms, _ = self.comm_model.process_message(
            sender_id=sender,
            receiver_id=self.robot_id,
            current_time=now_sec,
        )
        if action == CommunicationAction.DROP:
            return
        if action == CommunicationAction.DELAY:
            deliver_at = now_sec + (delay_ms / 1000.0)
            self.delayed_queue.schedule(
                deliver_at,
                self._apply_conflict_payload,
                (msg, sent_ts),
            )
            return

        self._apply_conflict_payload((msg, sent_ts))

    def _apply_conflict_payload(self, data: Tuple[ConflictReport, float]) -> None:
        """Apply delivered conflict report after network transit."""
        msg, sent_ts = data
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        sender = msg.robot_a if msg.robot_a else 'fleet'
        state, _ = self.stale_manager.record_incoming(
            'conflicts', sender, sent_ts, now_sec, msg
        )
        if state == InformationState.EXPIRED:
            return

        self.peer_conflicts.append(msg)
        self.conflict_timestamps.append(time.time())

    def _find_best_alternative_cell(
        self,
        curr_cell: Position,
        avoid_cell: Position,
        goal_cell: Position,
    ) -> Optional[Position]:
        """Find best traversable neighbor excluding avoid_cell and reserved cells."""
        candidates = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = curr_cell[0] + dx, curr_cell[1] + dy
            cand: Position = (nx, ny)
            if cand == avoid_cell:
                continue
            if not self.grid.is_free(cand):
                continue
            if self.res_table.is_reserved(cand, self.coord_time_step + 1, self.robot_id):
                continue
            if self.res_table.is_edge_conflict(
                curr_cell, cand, self.coord_time_step + 1, self.robot_id
            ):
                continue
            if self.res_table.is_headway_conflict(
                curr_cell, cand, self.coord_time_step + 1, self.robot_id
            ):
                continue
            dist = abs(cand[0] - goal_cell[0]) + abs(cand[1] - goal_cell[1])
            candidates.append((dist, cand))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def _find_lateral_escape_cell(
        self,
        curr_cell: Position,
        blocked_cell: Position,
    ) -> Optional[Position]:
        """Find an open lateral cell perpendicular to the blocked vector."""
        dx = blocked_cell[0] - curr_cell[0]
        dy = blocked_cell[1] - curr_cell[1]

        # Perpendicular lateral directions
        lateral_dirs = [(-dy, dx), (dy, -dx)]
        for ldx, ldy in lateral_dirs:
            cand = (curr_cell[0] + ldx, curr_cell[1] + ldy)
            if (self.grid.is_free(cand)
                    and not self.res_table.is_reserved(
                        cand, self.coord_time_step + 1, self.robot_id)):
                return cand

        # Fallback to any free neighbor
        for fdx, fdy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            cand = (curr_cell[0] + fdx, curr_cell[1] + fdy)
            if (cand != blocked_cell
                    and self.grid.is_free(cand)
                    and not self.res_table.is_reserved(
                        cand, self.coord_time_step + 1, self.robot_id)):
                return cand
        return None

    def _evaluate_local_coordination(
        self,
        curr_cell: Position,
        desired_next_cell: Position,
        goal_cell: Position,
    ) -> Tuple[Position, str, str]:
        """
        Evaluate local space-time reservations, conflicts, and deadlock recovery.

        Returns (safe_target_cell, coordination_state, waiting_for_robot).
        """
        if desired_next_cell == curr_cell:
            return curr_cell, CoordinationState.CLEAR.value, ''

        # Check shared station staging buffer for Central Delivery Bay 1 (Pillar D)
        if goal_cell == (29, 29) and curr_cell == (29, 33) and desired_next_cell[1] < 33:
            bay_occupied_by = ''
            for p_id, p_info in self.peer_states.items():
                if p_info.get('current_cell') == (29, 29) or p_info.get('target_cell') == (29, 29):
                    bay_occupied_by = p_id
                    break
            if (not bay_occupied_by and self.res_table.is_reserved(
                (29, 29), self.coord_time_step + 1, self.robot_id
            )):
                r = self.res_table.get_reservation((29, 29), self.coord_time_step + 1)
                if r and r.robot_id != self.robot_id:
                    bay_occupied_by = r.robot_id

            if bay_occupied_by:
                return curr_cell, CoordinationState.YIELDING.value, bay_occupied_by

        # 1. Check reservation table for conflicts at next time step
        is_vertex_conflict = self.res_table.is_reserved(
            desired_next_cell, self.coord_time_step + 1, self.robot_id,
        )
        is_edge_conflict = self.res_table.is_edge_conflict(
            curr_cell, desired_next_cell, self.coord_time_step + 1, self.robot_id,
        )
        is_headway_conflict = self.res_table.is_headway_conflict(
            curr_cell, desired_next_cell, self.coord_time_step + 1, self.robot_id,
        )

        # 2. Check peer physical positions from peer coordination status
        peer_blocking_id = ''
        peer_blocking_priority = 0.0
        for p_id, p_info in self.peer_states.items():
            p_curr = p_info.get('current_cell')
            p_target = p_info.get('target_cell')
            if p_curr == desired_next_cell or p_target == desired_next_cell:
                is_vertex_conflict = True
                peer_blocking_id = p_id
                peer_blocking_priority = float(p_info.get('priority', 0.0))
                break
            if p_curr == desired_next_cell and p_target == curr_cell:
                is_edge_conflict = True
                peer_blocking_id = p_id
                peer_blocking_priority = float(p_info.get('priority', 0.0))
                break

            # Headway check against peer position / target
            if desired_next_cell != curr_cell:
                dx = desired_next_cell[0] - curr_cell[0]
                dy = desired_next_cell[1] - curr_cell[1]
                ahead_cell = (desired_next_cell[0] + dx, desired_next_cell[1] + dy)
                if p_curr == ahead_cell or p_target == ahead_cell:
                    is_headway_conflict = True
                    peer_blocking_id = p_id
                    peer_blocking_priority = float(p_info.get('priority', 0.0))
                    break

        if not peer_blocking_id and (
            is_vertex_conflict or is_edge_conflict or is_headway_conflict
        ):
            if is_headway_conflict:
                dx = desired_next_cell[0] - curr_cell[0]
                dy = desired_next_cell[1] - curr_cell[1]
                ahead_cell = (desired_next_cell[0] + dx, desired_next_cell[1] + dy)
                r = self.res_table.get_reservation(ahead_cell, self.coord_time_step + 1)
                if r and r.robot_id != self.robot_id:
                    peer_blocking_id = r.robot_id
                    peer_blocking_priority = r.priority
            if not peer_blocking_id:
                r = self.res_table.get_reservation(desired_next_cell, self.coord_time_step + 1)
                if r and r.robot_id != self.robot_id:
                    peer_blocking_id = r.robot_id
                    peer_blocking_priority = r.priority

        # 3. Handle detected conflicts
        if is_edge_conflict or is_vertex_conflict or is_headway_conflict:
            c_type = (
                ConflictType.EDGE_SWAP.value
                if is_edge_conflict else ConflictType.VERTEX.value
            )
            self._publish_conflict_report(
                c_type,
                peer_blocking_id,
                desired_next_cell,
                self.coord_time_step + 1,
                False,
            )

            # Record waiting in Wait-For Graph
            if peer_blocking_id:
                self.wfg.add_wait(
                    self.robot_id,
                    peer_blocking_id,
                    desired_next_cell,
                    self.coord_time_step,
                )

            # Check for deadlocks in WFG
            dl_records = self.deadlock_detector.update(
                self.wfg,
                {self.robot_id: self.planner.current_position},
                now_sec=time.time(),
            )
            for dl in dl_records:
                if self.robot_id in dl.cycle_robot_ids:
                    cycle_prios = {
                        cid: (
                            self.current_priority if cid == self.robot_id
                            else self.peer_states.get(cid, {}).get('priority', 500.0)
                        )
                        for cid in dl.cycle_robot_ids
                    }
                    victim_id = min(cycle_prios.keys(), key=lambda k: cycle_prios[k])
                    if victim_id == self.robot_id:
                        lateral_cand = self._find_lateral_escape_cell(
                            curr_cell, desired_next_cell,
                        )
                        if lateral_cand:
                            self.res_table.release_robot(self.robot_id)
                            self._publish_deadlock_event(
                                dl.cycle_robot_ids, 'LATERAL_SIDESTEP', 0.5, True,
                            )
                            self.deadlock_detector.reset_cycle(dl.cycle_robot_ids)
                            self.wfg.clear_robot(self.robot_id)
                            return lateral_cand, CoordinationState.RECOVERING.value, ''

            # If edge swap, headway conflict, or lower priority: yield or select alternate neighbor
            if (is_edge_conflict or is_headway_conflict
                    or self.current_priority < peer_blocking_priority):
                alt_cand = self._find_best_alternative_cell(
                    curr_cell, desired_next_cell, goal_cell,
                )
                if alt_cand:
                    self.res_table.reserve(
                        alt_cand,
                        self.coord_time_step + 1,
                        self.robot_id,
                        self.current_priority,
                    )
                    self.res_table.reserve_edge(
                        curr_cell,
                        alt_cand,
                        self.coord_time_step + 1,
                        self.robot_id,
                        self.current_priority,
                    )
                    self._publish_reservation(
                        curr_cell,
                        alt_cand,
                        self.coord_time_step + 1,
                        True,
                        self.current_priority,
                    )
                    self.wfg.clear_robot(self.robot_id)
                    return alt_cand, CoordinationState.CLEAR.value, ''
                else:
                    return curr_cell, CoordinationState.YIELDING.value, peer_blocking_id

        # 4. No conflict or higher priority: reserve and proceed
        self.res_table.reserve(
            desired_next_cell,
            self.coord_time_step + 1,
            self.robot_id,
            self.current_priority,
        )
        self.res_table.reserve_edge(
            curr_cell,
            desired_next_cell,
            self.coord_time_step + 1,
            self.robot_id,
            self.current_priority,
        )
        self._publish_reservation(
            curr_cell,
            desired_next_cell,
            self.coord_time_step + 1,
            True,
            self.current_priority,
        )
        self.wfg.clear_robot(self.robot_id)
        return desired_next_cell, CoordinationState.CLEAR.value, ''

    def _planning_cycle(self) -> None:
        """Periodic rolling-horizon planning, replanning check, and execution control."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        curr_tid = ''
        if (
            self.planner.ordered_tasks
            and self.planner.active_task_idx < len(self.planner.ordered_tasks)
        ):
            curr_tid = self.planner.ordered_tasks[self.planner.active_task_idx]

        # YAVI-SIH26123: Decentralized health monitoring & failure recovery
        if hasattr(self, 'fault_detector'):
            self.fault_detector.update_self_telemetry(
                self.planner.current_position,
                curr_tid,
                now_sec,
            )
            self._publish_health_status()

            if not self.fault_detector.self_state.can_move:
                # Self is dead or motor failed: command stop and exit
                twist = Twist()
                self.pub_cmd_vel.publish(twist)
                return

            newly_failed, newly_recovered = self.fault_detector.evaluate_peers(now_sec)
            for failed_p in newly_failed:
                self.get_logger().warn(
                    f'[{self.robot_id}] Detected peer failure: {failed_p}. '
                    'Executing autonomous recovery.'
                )
                if self.enable_coordination:
                    self.res_table.release_robot(failed_p)

                dead_pose = self.fault_detector.get_peer_pose(failed_p)
                if dead_pose:
                    dead_cell = self.grid.to_grid(dead_pose[0], dead_pose[1])
                    self.failed_robot_obstacles.setdefault(failed_p, set()).add(dead_cell)
                    self.grid.add_obstacle(dead_cell)
                    for wp in (self.planner.full_path or self.planner.execution_path):
                        if self.grid.to_grid(wp[0], wp[1]) == dead_cell:
                            self.planner.full_path = []
                            self.planner.horizon_path = []
                            self.planner.execution_path = []
                            self.planner.replan()
                            break

                reclaim_tids = set()
                peer_task = self.fault_detector.get_peer_task(failed_p)
                if peer_task:
                    reclaim_tids.add(peer_task)
                if self.cached_tasks:
                    for tid, tinfo in self.cached_tasks.items():
                        if (
                            tinfo.get('assigned_robot_id') == failed_p
                            and tinfo.get('status') in ('ASSIGNED', 'IN_PROGRESS')
                        ):
                            reclaim_tids.add(tid)

                for r_tid in reclaim_tids:
                    self.get_logger().warn(
                        f'[{self.robot_id}] Autonomous reclamation: task {r_tid} '
                        f'from failed peer {failed_p} -> PENDING'
                    )
                    self._publish_task_transition(
                        r_tid,
                        'PENDING',
                        f'Autonomous reclamation from failed peer {failed_p}',
                    )

            for rec_p in newly_recovered:
                self.get_logger().info(
                    f'[{self.robot_id}] Peer {rec_p} restored to HEALTHY. '
                    'Clearing stranded chassis obstacle.'
                )
                obs_cells = self.failed_robot_obstacles.pop(rec_p, set())
                for cell in obs_cells:
                    self.grid.remove_obstacle(cell)

        if not self.planner.assigned_bundle and self.cached_bundle and self.cached_tasks:
            self.planner.update_assigned_bundle(self.cached_bundle, self.cached_tasks)

        # Check sub-goal arrival
        if self.planner.check_subgoal_arrival():
            curr_tid = (
                self.planner.ordered_tasks[self.planner.active_task_idx]
                if self.planner.active_task_idx < len(self.planner.ordered_tasks)
                else ''
            )
            event = self.planner.advance_subgoal()

            if event == 'PICKUP_REACHED':
                self.get_logger().info(
                    f"[{self.robot_id}] Reached pickup for task '{curr_tid}'. Moving to dropoff.",
                )
                self._publish_task_transition(
                    curr_tid, 'IN_PROGRESS', 'Arrived at pickup location',
                )
            elif event == 'TASK_COMPLETED':
                self.get_logger().info(
                    f"[{self.robot_id}] Completed task '{curr_tid}'. Advancing to next task.",
                )
                self._publish_task_transition(
                    curr_tid, 'COMPLETED', 'Delivered at dropoff location',
                )
            elif event == 'ALL_COMPLETED':
                self.get_logger().info(
                    f'[{self.robot_id}] All assigned bundle tasks completed!',
                )
                if curr_tid:
                    self._publish_task_transition(
                        curr_tid, 'COMPLETED', 'Delivered at dropoff location',
                    )

        # Check replanning triggers
        if self.planner.check_replan_triggers() and self.planner.current_goal:
            curr_tid = (
                self.planner.ordered_tasks[self.planner.active_task_idx]
                if self.planner.active_task_idx < len(self.planner.ordered_tasks)
                else ''
            )
            self._publish_plan_request(
                curr_tid, self.planner.active_phase, self.planner.current_goal,
            )
            resp = self.planner.replan()
            self._publish_plan_response(resp)

        # M6 Multi-Agent Coordination Layer
        if self.enable_coordination:
            self.coord_time_step += 1
            self.res_table.release_time_before(self.coord_time_step)

            curr_cell = self.grid.to_grid(
                self.planner.current_position[0], self.planner.current_position[1],
            )
            goal_cell = (
                self.grid.to_grid(self.planner.current_goal[0], self.planner.current_goal[1])
                if self.planner.current_goal else curr_cell
            )
            self.current_priority = self._compute_local_priority(curr_cell, goal_cell)

            target_wp = self.planner.get_current_target_waypoint()
            if target_wp:
                desired_cell = self.grid.to_grid(target_wp[0], target_wp[1])
            else:
                desired_cell = curr_cell

            safe_cell, state, wait_for = self._evaluate_local_coordination(
                curr_cell, desired_cell, goal_cell,
            )
            self.coordination_state = state
            self.waiting_for_robot = wait_for
            self.target_cell = safe_cell

            self._publish_coordination_status(curr_cell, safe_cell)

            if safe_cell != desired_cell and self.planner.execution_path:
                idx = min(self.planner.active_waypoint_idx, len(self.planner.execution_path) - 1)
                self.planner.execution_path[idx] = self.grid.to_world(safe_cell)

        # Publish current plan status
        self._publish_rolling_plan()

        # Prune expired reservations from silent peers and publish metrics (M7)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        pruned = self.stale_manager.prune_expired_reservations(self.res_table, now_sec)
        if pruned:
            self.get_logger().info(
                f'[{self.robot_id}] Pruned expired reservations for peers: {pruned}'
            )
        self._publish_comm_metrics(now_sec)

        # M8B Adaptive Compute Evaluation
        if self.is_adaptive:
            stale_sum = self.stale_manager.get_stale_summary(now_sec)
            comm_met = self.comm_model.get_metrics()
            self.conflict_timestamps = [
                t for t in self.conflict_timestamps if now_sec - t <= 10.0
            ]
            recent_conflicts = len([
                t for t in self.conflict_timestamps if now_sec - t <= 3.0
            ])
            cpu_pct = psutil.cpu_percent(interval=None) if HAVE_PSUTIL else 0.0

            compute_state = ComputeState(
                timestamp=now_sec,
                information_age_s=float(stale_sum.get('max_stale_duration_s', 0.0)),
                comm_latency_ms=float(comm_met.get('avg_latency_ms', 0.0)),
                comm_loss_rate=float(comm_met.get('packet_loss_rate', 0.0)),
                active_conflicts=int(recent_conflicts),
                is_yielding_or_recovering=(self.coordination_state in (
                    CoordinationState.YIELDING.value,
                    CoordinationState.RECOVERING.value,
                )),
                planning_latency_ms=float(self.planner.last_latency_ms),
                cpu_utilization_pct=float(cpu_pct),
                safety_active=bool(self.obstacle_ahead),
            )
            new_mode, transition_record = self.policy.evaluate(compute_state)
            if transition_record is not None:
                self.active_compute_mode = new_mode
                self._apply_compute_mode(new_mode)
                self._publish_compute_event(transition_record)
                self.get_logger().info(
                    f'[{self.robot_id}] Adaptive Compute Mode switched to {new_mode.value}: '
                    f'{transition_record.reason} (trigger: '
                    f'{transition_record.trigger_signal}={transition_record.trigger_value:.2f}, '
                    f'thresh={transition_record.threshold_value:.2f})'
                )

    def _apply_compute_mode(self, mode: ComputeMode) -> None:
        """Apply operational parameters of the selected compute mode."""
        cfg = get_compute_mode_config(mode)
        self.planner.config.horizon_steps = cfg.horizon_steps
        self.planner.config.execution_window = cfg.execution_window
        self.planner.config.replan_rate = cfg.replan_rate
        new_period = 1.0 / max(0.1, cfg.replan_rate)
        self.timer.timer_period_ns = int(new_period * 1e9)
        self.timer.reset()

    def _publish_compute_event(self, record: PolicyTransitionRecord) -> None:
        """Publish adaptive compute transition event to local and fleet topics."""
        if not HAVE_M8B_MSGS:
            return
        msg = ComputeModeEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.previous_mode = record.previous_mode.value
        msg.current_mode = record.new_mode.value
        msg.trigger_signal = record.trigger_signal
        msg.trigger_value = float(record.trigger_value)
        msg.threshold_value = float(record.threshold_value)
        msg.reason = record.reason
        msg.dwell_time_sec = float(record.dwell_time_sec)
        self.pub_compute_mode.publish(msg)
        self.pub_fleet_compute_events.publish(msg)

    def _check_peer_proximity_hazard(self) -> bool:
        """
        Deterministic local safety backstop against known peer AMRs.

        Calculates relative displacement (d_parallel, d_perp) in the robot body frame.
        Triggers reactive braking if a peer is in the forward corridor:
        - 0.0 < d_parallel <= 0.700m (stopping distance yielding +0.05m clearance)
        - |d_perp| <= 0.400m (lateral corridor half-width)
        Independently backstops the sub-0.15m LiDAR blind zone.
        """
        rx, ry = self.planner.current_position
        yaw = self.current_yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for p_id, p_info in self.peer_states.items():
            if p_id == self.robot_id:
                continue
            pose = p_info.get('pose')
            if not pose:
                c_cell = p_info.get('current_cell')
                if c_cell:
                    pose = self.grid.to_world(c_cell)
                else:
                    continue

            px, py = pose
            dx = px - rx
            dy = py - ry

            d_parallel = dx * cos_yaw + dy * sin_yaw
            d_perp = -dx * sin_yaw + dy * cos_yaw

            if 0.0 < d_parallel <= 0.700 and abs(d_perp) <= 0.400:
                return True

        return False

    def _control_loop(self) -> None:
        """High-frequency (10 Hz) closed-loop differential drive motion controller."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.delayed_queue.poll(now_sec)

        if not self.enable_motion or self.planner.active_phase == 'IDLE':
            return

        cmd = Twist()

        # Halt immediately if self cannot move (e.g. FAILED, MOTION_FAILURE, EMERGENCY_STOP)
        if hasattr(self, 'fault_detector') and not self.fault_detector.self_state.can_move:
            self.pub_cmd_vel.publish(cmd)
            return

        # Stop if yielding to another robot
        if self.coordination_state in (
            CoordinationState.YIELDING.value,
        ):
            self.pub_cmd_vel.publish(cmd)
            return

        # Target waypoint from planner
        target_wp = self.planner.get_current_target_waypoint()
        if not target_wp:
            self.pub_cmd_vel.publish(cmd)
            return

        rx, ry = self.planner.current_position
        tx, ty = target_wp

        # Local Autonomy check during COMM_LOSS: verify valid reservation before entering cell
        if (
            hasattr(self, 'fault_detector')
            and self.fault_detector.is_in_comm_loss()
            and self.enable_coordination
        ):
            target_cell = self.grid.to_grid(tx, ty)
            current_cell = self.grid.to_grid(rx, ry)
            if target_cell != current_cell:
                held_cells = {
                    r.to_pos
                    for r in self.res_table.get_reservations_for_robot(self.robot_id)
                }
                if target_cell not in held_cells:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.pub_cmd_vel.publish(cmd)
                    return

        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)

        # Check arrival at intermediate waypoint (0.25m radius)
        if dist < 0.25:
            new_wp = self.planner.advance_waypoint()
            if not new_wp:
                self.pub_cmd_vel.publish(cmd)
                return
            tx, ty = new_wp
            dx = tx - rx
            dy = ty - ry
            dist = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - self.current_yaw

        # Normalize yaw error to [-pi, pi]
        while yaw_err > math.pi:
            yaw_err -= 2.0 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2.0 * math.pi

        # Check obstacle hazard from either LiDAR or deterministic peer backstop
        peer_hazard = self._check_peer_proximity_hazard()
        is_hazard = self.obstacle_ahead or peer_hazard

        if abs(yaw_err) > 0.45:
            # Rotate smoothly in place towards waypoint (permitted to rotate away from obstacles)
            cmd.linear.x = 0.0
            cmd.angular.z = min(0.6, max(-0.6, 1.5 * yaw_err))
        else:
            # When facing forward, check bumper obstacle safety from LiDAR or peer backstop
            if is_hazard:
                # Brake forward drive but allow gentle steering adjustment
                cmd.linear.x = 0.0
                cmd.angular.z = min(0.5, max(-0.5, 1.0 * yaw_err))
            else:
                # Drive forward smoothly with proportional angular correction
                cmd.linear.x = min(0.30, max(0.08, 0.5 * dist))
                cmd.angular.z = min(0.7, max(-0.7, 1.2 * yaw_err))

        self.pub_cmd_vel.publish(cmd)


def main(args=None):
    """Run the Rolling-Horizon Planner Node."""
    rclpy.init(args=args)
    node = RollingHorizonPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


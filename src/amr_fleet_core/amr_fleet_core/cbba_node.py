"""Decentralized ROS 2 Node executing CBBA for a single AMR."""

from typing import Any, Dict, Set, Tuple

from amr_fleet_core.cbba_agent import CBBAAgent, CBBAConfig
from amr_fleet_core.communication_model import (
    CommunicationAction,
    CommunicationImpairmentModel,
    CommunicationProfileConfig,
    DelayedMessageQueue,
    PRESET_PROFILES,
)
from amr_fleet_core.compute_modes import ComputeMode, get_compute_mode_config
from amr_fleet_core.fault_detector import FaultDetector
from amr_fleet_core.stale_state_manager import (
    InformationState,
    StaleStateManager,
)
from amr_fleet_msgs.msg import (
    CBBABid,
    RobotBundle,
    TaskEvent as TaskEventMsg,
    TaskList,
)
try:
    from amr_fleet_msgs.msg import CommunicationProfile
    HAVE_M7_MSGS = True
except ImportError:
    HAVE_M7_MSGS = False
try:
    from amr_fleet_msgs.msg import ComputeModeEvent
    HAVE_M8B_MSGS = True
except ImportError:
    HAVE_M8B_MSGS = False
try:
    from amr_fleet_msgs.msg import RobotHealth
    HAVE_HEALTH_MSG = True
except ImportError:
    HAVE_HEALTH_MSG = False
from builtin_interfaces.msg import Time as BuiltinTime
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def float_to_builtin_time(sec_float: float) -> BuiltinTime:
    """Convert float timestamp to builtin_interfaces/Time msg."""
    msg = BuiltinTime()
    if sec_float <= 0.0:
        msg.sec = 0
        msg.nanosec = 0
    else:
        msg.sec = int(sec_float)
        msg.nanosec = int((sec_float - int(sec_float)) * 1e9)
    return msg


class CBBANode(Node):
    """Decentralized ROS 2 node executing CBBA bundle building and consensus."""

    def __init__(self) -> None:
        """Initialize CBBANode parameters, pubs/subs, and timers."""
        super().__init__('cbba_node')

        # Parameters
        self.declare_parameter('robot_id', '')
        self.declare_parameter('max_bundle_size', 4)
        self.declare_parameter('weight_priority', 100.0)
        self.declare_parameter('weight_distance', 10.0)
        self.declare_parameter('weight_late', 5.0)
        self.declare_parameter('discount_factor', 0.95)
        self.declare_parameter('consensus_rate', 5.0)
        self.declare_parameter('stable_rounds_for_convergence', 5)
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

        robot_id_param = self.get_parameter('robot_id').get_parameter_value().string_value
        if not robot_id_param:
            # Fall back to namespace stripped of leading slashes
            ns = self.get_namespace().strip('/')
            robot_id_param = ns if ns else 'amr_0'
        self.robot_id = robot_id_param

        raw_mode = str(self.get_parameter('compute_mode').value)
        if raw_mode.upper() in ('ADAPTIVE', 'COMPUTE_MODE_ADAPTIVE'):
            self.compute_mode = 'ADAPTIVE'
        else:
            self.compute_mode = ComputeMode.from_string(raw_mode)

        max_bundle = self.get_parameter('max_bundle_size').get_parameter_value().integer_value
        w_prio = self.get_parameter('weight_priority').get_parameter_value().double_value
        w_dist = self.get_parameter('weight_distance').get_parameter_value().double_value
        w_late = self.get_parameter('weight_late').get_parameter_value().double_value
        discount = self.get_parameter('discount_factor').get_parameter_value().double_value
        rate = self.get_parameter('consensus_rate').get_parameter_value().double_value

        if self.compute_mode != 'ADAPTIVE' and self.compute_mode != ComputeMode.NORMAL:
            mode_cfg = get_compute_mode_config(self.compute_mode)
            rate = mode_cfg.consensus_rate

        self.stable_thresh = self.get_parameter(
            'stable_rounds_for_convergence'
        ).get_parameter_value().integer_value

        config = CBBAConfig(
            max_bundle_size=int(max_bundle),
            weight_priority=float(w_prio),
            weight_distance=float(w_dist),
            weight_late=float(w_late),
            discount_factor=float(discount),
        )
        self.agent = CBBAAgent(robot_id=self.robot_id, config=config)

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

        # Internal state
        self.task_pool: Dict[str, Any] = {}
        self.task_all_states: Dict[str, str] = {}
        self.task_metadata_cache: Dict[str, Any] = {}
        self.iteration: int = 0
        self.consecutive_stable_rounds: int = 0
        self.is_converged: bool = False
        self.assigned_tasks_committed: Set[str] = set()

        # Publishers
        self.pub_bids = self.create_publisher(CBBABid, '/fleet/cbba_bids', 50)
        self.pub_bundle = self.create_publisher(
            RobotBundle, f'/{self.robot_id}/bundle', 10
        )
        self.pub_task_status = self.create_publisher(
            TaskEventMsg, '/tasks/update_status', 20
        )

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self._handle_odom,
            10,
        )
        self.sub_tasks = self.create_subscription(
            TaskList,
            '/tasks/available',
            self._handle_available_tasks,
            10,
        )
        self.sub_tasks_all = self.create_subscription(
            TaskList,
            '/tasks/all',
            self._handle_all_tasks,
            10,
        )
        self.sub_bids = self.create_subscription(
            CBBABid,
            '/fleet/cbba_bids',
            self._handle_peer_bid,
            50,
        )
        if HAVE_M7_MSGS:
            self.sub_comm_profile = self.create_subscription(
                CommunicationProfile,
                '/fleet/comm_profile',
                self._handle_comm_profile,
                10,
            )
        if HAVE_M8B_MSGS and self.compute_mode == 'ADAPTIVE':
            self.sub_compute_mode = self.create_subscription(
                ComputeModeEvent,
                f'/{self.robot_id}/compute_mode',
                self._handle_compute_mode_event,
                10,
            )

        # YAVI-SIH26123 Fault Resilience Layer
        self.fault_detector = FaultDetector(self.robot_id)
        if HAVE_HEALTH_MSG:
            self.sub_fleet_health = self.create_subscription(
                RobotHealth,
                '/fleet/robot_health',
                self._handle_robot_health,
                20,
            )

        # Periodic timer
        timer_period = 1.0 / max(0.1, rate)
        self.timer = self.create_timer(timer_period, self._consensus_cycle)

        self.get_logger().info(
            f'CBBA Node initialized for {self.robot_id} (bundle cap: {max_bundle})'
        )

    def _handle_robot_health(self, msg: Any) -> None:
        """Process peer robot health telemetry."""
        if msg.robot_id == self.robot_id:
            return
        pose = (float(msg.last_pose.x), float(msg.last_pose.y))
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.fault_detector.record_peer_heartbeat(
            msg.robot_id,
            msg.health_state,
            pose,
            msg.active_task_id,
            now_sec,
        )

    def _handle_odom(self, msg: Odometry) -> None:
        """Update agent position from odometry."""
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.agent.update_position(pos)

    def _handle_available_tasks(self, msg: TaskList) -> None:
        """Cache incoming pool of available tasks."""
        current_map: Dict[str, Any] = {}
        for td in msg.tasks:
            t_id = td.task_id
            current_map[t_id] = {
                'task_id': t_id,
                'pickup': (td.pickup_pose.x, td.pickup_pose.y),
                'dropoff': (td.dropoff_pose.x, td.dropoff_pose.y),
                'priority': td.priority,
                'deadline': (
                    td.deadline.sec + td.deadline.nanosec * 1e-9
                    if td.deadline.sec > 0 else None
                ),
                'requested_robot': getattr(td, 'requested_robot', '').strip(),
            }
        self.task_pool = current_map

    def _handle_all_tasks(self, msg: TaskList) -> None:
        """Cache fleet-wide task definitions, lifecycle states, and prune cancelled tasks."""
        cancelled_tasks = set()
        for td in msg.tasks:
            t_id = td.task_id
            self.task_all_states[t_id] = td.status
            self.task_metadata_cache[t_id] = {
                'task_id': t_id,
                'pickup': (td.pickup_pose.x, td.pickup_pose.y),
                'dropoff': (td.dropoff_pose.x, td.dropoff_pose.y),
                'priority': td.priority,
                'status': td.status,
                'requested_robot': getattr(td, 'requested_robot', ''),
                'deadline': (
                    td.deadline.sec + td.deadline.nanosec * 1e-9
                    if td.deadline.sec > 0 else None
                ),
            }
            if td.status.upper() == 'CANCELLED':
                cancelled_tasks.add(t_id)

        # Prune cancelled tasks from our local bundle and winning beliefs so CBBA
        # does not re-commit or hold stale claims on operator-cancelled tasks.
        if cancelled_tasks:
            tasks_to_remove = cancelled_tasks & set(self.agent.state.bundle)
            if tasks_to_remove:
                self.agent.state.bundle = [
                    t for t in self.agent.state.bundle if t not in tasks_to_remove
                ]
                for t in tasks_to_remove:
                    self.agent.state.winning_robots.pop(t, None)
                    self.agent.state.winning_bids.pop(t, None)
                    self.agent.state.timestamps.pop(t, None)
                    self.assigned_tasks_committed.discard(t)
                if tasks_to_remove:
                    full_map = {**self.task_metadata_cache, **self.task_pool}
                    self.agent._rebuild_path(full_map)

    def _handle_comm_profile(self, msg: Any) -> None:
        """Dynamically update communication degradation profile across the fleet."""
        self.get_logger().info(
            f'[{self.robot_id}] Received comm profile update in CBBA: '
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

    def _broadcast_bids(self) -> None:
        """Broadcast current bid vector immediately."""
        bid_msg = CBBABid()
        bid_msg.header.stamp = self.get_clock().now().to_msg()
        bid_msg.robot_id = self.robot_id
        bid_msg.iteration = self.iteration

        all_task_ids = sorted(self.agent.state.winning_bids.keys())
        bid_msg.task_ids = all_task_ids
        bid_msg.winning_bids = [
            float(self.agent.state.winning_bids[t]) for t in all_task_ids
        ]
        bid_msg.winning_robots = [
            str(self.agent.state.winning_robots.get(t, '')) for t in all_task_ids
        ]
        bid_msg.timestamps = [
            float(self.agent.state.timestamps.get(t, 0.0)) for t in all_task_ids
        ]
        self.pub_bids.publish(bid_msg)

    def _handle_peer_bid(self, msg: CBBABid) -> None:
        """Process peer bid vector through communication degradation boundary."""
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
                self._apply_peer_bid,
                (msg, sent_ts),
            )
            return

        self._apply_peer_bid((msg, sent_ts))

    def _apply_peer_bid(self, data: Tuple[CBBABid, float]) -> None:
        """Apply delivered peer bid vector after network transit."""
        msg, sent_ts = data
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        state, is_reconn = self.stale_manager.record_incoming(
            'cbba_bids', msg.robot_id, sent_ts, now_sec, msg
        )
        if is_reconn:
            self._broadcast_bids()

        if state == InformationState.EXPIRED:
            return

        peer_bids: Dict[str, float] = {}
        peer_robots: Dict[str, str] = {}
        peer_times: Dict[str, float] = {}

        for idx, t_id in enumerate(msg.task_ids):
            if idx < len(msg.winning_bids):
                peer_bids[t_id] = float(msg.winning_bids[idx])
            if idx < len(msg.winning_robots):
                peer_robots[t_id] = str(msg.winning_robots[idx])
            if idx < len(msg.timestamps):
                peer_times[t_id] = float(msg.timestamps[idx])

        full_map = {**self.task_metadata_cache, **self.task_pool}
        locked_tasks = {
            t for t in self.agent.state.bundle
            if self.task_all_states.get(t) in ('IN_PROGRESS', 'ASSIGNED')
            or t in self.assigned_tasks_committed
        }

        changed = self.agent.resolve_conflicts(
            peer_id=msg.robot_id,
            peer_iteration=msg.iteration,
            peer_winning_bids=peer_bids,
            peer_winning_robots=peer_robots,
            peer_timestamps=peer_times,
            task_map=full_map,
            current_time=now_sec,
            locked_tasks=locked_tasks,
        )

        if changed:
            self.consecutive_stable_rounds = 0
            self.is_converged = False

    def _consensus_cycle(self) -> None:
        """Periodic CBBA bundle update and broadcast loop."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.delayed_queue.poll(now_sec)
        self.iteration += 1

        # YAVI-SIH26123: Evaluate peer failures and purge stale peer winning beliefs
        if hasattr(self, 'fault_detector'):
            self.fault_detector.update_self_telemetry(self.agent.position, '', now_sec)
            if not self.fault_detector.is_self_healthy():
                # Self is failed; do not build bundle or broadcast winning bids
                return

            newly_failed, newly_recovered = self.fault_detector.evaluate_peers(now_sec)
            for failed_p in newly_failed:
                freed = self.agent.purge_failed_peer_tasks(failed_p, now_sec)
                if freed:
                    self.get_logger().info(
                        f'[{self.robot_id}] Purged tasks {freed} '
                        f'previously held by failed peer {failed_p}'
                    )
                    self.consecutive_stable_rounds = 0
                    self.is_converged = False

            for rec_p in newly_recovered:
                self.get_logger().info(
                    f'[{self.robot_id}] Peer {rec_p} recovered. Triggering auction update.'
                )
                self.consecutive_stable_rounds = 0
                self.is_converged = False

        # Free completed, cancelled, or failed tasks from bundle to release capacity
        stale_or_done = [
            t for t in self.agent.state.bundle
            if self.task_all_states.get(t) in ('COMPLETED', 'CANCELLED', 'FAILED')
        ]
        # Also release any task in bundle that was externally requeued to PENDING
        requeued_in_bundle = [
            t for t in self.agent.state.bundle
            if t in self.assigned_tasks_committed and self.task_all_states.get(t) == 'PENDING'
        ]
        to_purge = set(stale_or_done + requeued_in_bundle)
        if to_purge:
            self.agent.state.bundle = [
                t for t in self.agent.state.bundle
                if t not in to_purge
            ]
            for t in to_purge:
                self.assigned_tasks_committed.discard(t)
                self.agent.state.winning_bids.pop(t, None)
                self.agent.state.winning_robots.pop(t, None)
            full_map = {**self.task_metadata_cache, **self.task_pool}
            self.agent._rebuild_path(full_map)
            self.consecutive_stable_rounds = 0
            self.is_converged = False

        # Locked tasks are those already in-progress or committed
        locked_tasks = {
            t for t in self.agent.state.bundle
            if self.task_all_states.get(t) in ('IN_PROGRESS', 'ASSIGNED')
            or t in self.assigned_tasks_committed
        }

        full_map = {**self.task_metadata_cache, **self.task_pool}
        candidate_map = {
            t_id: data for t_id, data in self.task_pool.items()
            if self.task_all_states.get(t_id, 'PENDING') == 'PENDING'
        }

        # Phase 1: Build bundle if candidate tasks are available
        added = 0
        if candidate_map:
            added = self.agent.build_bundle(
                candidate_map,
                current_time=now_sec,
                locked_tasks=locked_tasks,
                full_task_map=full_map,
            )

        if added > 0:
            self.consecutive_stable_rounds = 0
            self.is_converged = False
        else:
            self.consecutive_stable_rounds += 1

        if (
            self.consecutive_stable_rounds >= self.stable_thresh
            and len(self.agent.state.bundle) > 0
        ):
            self.is_converged = True

        # Phase 2: Broadcast current bid beliefs
        bid_msg = CBBABid()
        bid_msg.header.stamp = self.get_clock().now().to_msg()
        bid_msg.robot_id = self.robot_id
        bid_msg.iteration = self.iteration

        all_task_ids = sorted(self.agent.state.winning_bids.keys())
        bid_msg.task_ids = all_task_ids
        bid_msg.winning_bids = [
            float(self.agent.state.winning_bids[t]) for t in all_task_ids
        ]
        bid_msg.winning_robots = [
            str(self.agent.state.winning_robots.get(t, '')) for t in all_task_ids
        ]
        bid_msg.timestamps = [
            float(self.agent.state.timestamps.get(t, 0.0)) for t in all_task_ids
        ]
        self.pub_bids.publish(bid_msg)

        # Phase 3: Publish bundle status for observability
        bundle_msg = RobotBundle()
        bundle_msg.header.stamp = self.get_clock().now().to_msg()
        bundle_msg.robot_id = self.robot_id
        bundle_msg.task_ids = list(self.agent.state.bundle)
        bundle_msg.bid_values = [
            float(self.agent.state.winning_bids.get(t, 0.0))
            for t in self.agent.state.bundle
        ]
        bundle_msg.is_converged = self.is_converged
        self.pub_bundle.publish(bundle_msg)

        # Phase 4: Commit task transitions to TaskManager if converged
        if self.is_converged:
            for t_id in self.agent.state.bundle:
                if t_id not in self.assigned_tasks_committed:
                    event_msg = TaskEventMsg()
                    event_msg.header.stamp = self.get_clock().now().to_msg()
                    event_msg.task_id = t_id
                    event_msg.event_type = 'ASSIGNED'
                    event_msg.new_state = 'ASSIGNED'
                    event_msg.robot_id = self.robot_id
                    event_msg.timestamp = float_to_builtin_time(now_sec)
                    event_msg.details = 'Allocated via decentralized CBBA consensus'
                    self.pub_task_status.publish(event_msg)
                    self.assigned_tasks_committed.add(t_id)
                    self.get_logger().info(
                        f'Committed task {t_id} assignment to {self.robot_id}'
                    )

    def _handle_compute_mode_event(self, msg: Any) -> None:
        """Dynamically adjust CBBA consensus rate based on adaptive compute transitions."""
        new_mode = ComputeMode.from_string(msg.current_mode)
        cfg = get_compute_mode_config(new_mode)
        new_period = 1.0 / max(0.1, cfg.consensus_rate)
        self.timer.timer_period_ns = int(new_period * 1e9)
        self.timer.reset()
        self.get_logger().info(
            f'[{self.robot_id}] CBBA consensus rate updated to '
            f'{cfg.consensus_rate:.1f} Hz ({new_mode.value})'
        )


def main(args=None) -> None:
    """Run CBBA node."""
    rclpy.init(args=args)
    node = CBBANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


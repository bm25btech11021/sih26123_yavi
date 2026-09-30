import math
import os
import re
from typing import Dict, Optional

from amr_fleet_core.task_model import (
    InvalidTaskTransitionError,
    Task,
    TaskEvent,
    TaskLifecycleState,
    TaskPriority,
)
from amr_fleet_core.workload import WorkloadManager
from amr_fleet_msgs.msg import TaskDefinition, TaskEvent as TaskEventMsg, TaskList
from amr_fleet_msgs.srv import ControlTask, CreateTask
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node


def float_to_builtin_time(sec_float: Optional[float]) -> BuiltinTime:
    """Convert float seconds to builtin_interfaces/Time msg."""
    msg = BuiltinTime()
    if sec_float is None or sec_float <= 0.0:
        msg.sec = 0
        msg.nanosec = 0
    else:
        msg.sec = int(sec_float)
        msg.nanosec = int((sec_float - int(sec_float)) * 1e9)
    return msg


class TaskManagerNode(Node):
    """ROS 2 Node orchestrating task pools, publishing state, and logging events."""

    def __init__(self) -> None:
        super().__init__('amr_task_manager')

        # Declare parameters
        self.declare_parameter('workload_file', '')
        self.declare_parameter('publish_rate', 1.0)

        workload_file = self.get_parameter('workload_file').get_parameter_value().string_value
        pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Locate default workload if not specified
        if not workload_file:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            ws_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
            candidate = os.path.join(
                ws_root, 'config', 'workloads', 'workload_small_deterministic.yaml'
            )
            if os.path.isfile(candidate):
                workload_file = candidate

        self.tasks: Dict[str, Task] = {}

        if workload_file and os.path.isfile(workload_file):
            loaded = WorkloadManager.load_from_yaml(workload_file)
            self.tasks = {t.task_id: t for t in loaded}
            self.get_logger().info(
                f'Loaded {len(self.tasks)} tasks from workload: {workload_file}'
            )
        else:
            self.get_logger().warn(
                f"No workload file found at '{workload_file}'. Starting with empty pool."
            )

        # Publishers
        self.pub_all_tasks = self.create_publisher(TaskList, '/tasks/all', 10)
        self.pub_available_tasks = self.create_publisher(TaskList, '/tasks/available', 10)
        self.pub_events = self.create_publisher(TaskEventMsg, '/tasks/events', 20)

        # Subscriber for state transitions
        self.sub_status_update = self.create_subscription(
            TaskEventMsg,
            '/tasks/update_status',
            self._handle_status_update,
            10,
        )
        self.sub_mission_start = self.create_subscription(
            TaskEventMsg,
            '/tasks/start_mission',
            self._handle_mission_start,
            10,
        )

        self.mission_start_time: Optional[float] = None
        self.current_sim_time: float = 0.0
        self._task_counter: int = 1

        # Services for operator task management
        self.srv_create_task = self.create_service(
            CreateTask,
            '/tasks/create',
            self._handle_create_task,
        )
        self.srv_control_task = self.create_service(
            ControlTask,
            '/tasks/control',
            self._handle_control_task,
        )
        self.srv_cancel_task = self.create_service(
            ControlTask,
            '/tasks/cancel',
            self._handle_cancel_service,
        )
        self.srv_requeue_task = self.create_service(
            ControlTask,
            '/tasks/requeue',
            self._handle_requeue_service,
        )

        # Periodic publication timer
        timer_period = 1.0 / max(0.1, pub_rate)
        self.timer = self.create_timer(timer_period, self._publish_task_lists)

        self.get_logger().info('Task Manager Node initialized with operator control services.')

    def _handle_mission_start(self, msg: Optional[TaskEventMsg] = None) -> None:
        """Anchor mission start time to synchronize dynamic task release with benchmark clock."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.mission_start_time = now_sec
        self.get_logger().info(f'Mission clock anchored at t={now_sec:.2f}s')

    def task_to_msg(self, task: Task) -> TaskDefinition:
        """Convert a domain Task object to a ROS 2 TaskDefinition msg."""
        msg = TaskDefinition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task.task_id

        msg.pickup_pose = Point()
        msg.pickup_pose.x = float(task.pickup[0])
        msg.pickup_pose.y = float(task.pickup[1])
        msg.pickup_pose.z = 0.0

        msg.dropoff_pose = Point()
        msg.dropoff_pose.x = float(task.dropoff[0])
        msg.dropoff_pose.y = float(task.dropoff[1])
        msg.dropoff_pose.z = 0.0

        msg.priority = int(task.priority)
        msg.created_at = float_to_builtin_time(task.created_at)
        msg.deadline = float_to_builtin_time(task.deadline)
        msg.status = task.state.value
        msg.assigned_robot_id = task.assigned_robot_id or ''
        msg.requested_robot = task.requested_robot or ''
        return msg

    def event_to_msg(self, event: TaskEvent, task_id: str) -> TaskEventMsg:
        """Convert a domain TaskEvent to a ROS 2 TaskEvent msg."""
        msg = TaskEventMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.event_type = event.event_type
        msg.previous_state = event.from_state.value if event.from_state else ''
        msg.new_state = event.to_state.value
        msg.robot_id = event.robot_id or ''
        msg.timestamp = float_to_builtin_time(event.timestamp)
        msg.details = event.details
        return msg

    def _publish_task_lists(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.mission_start_time is None:
            self.mission_start_time = now_sec

        elapsed_sec = max(0.0, now_sec - self.mission_start_time)
        self.current_sim_time = elapsed_sec

        # Check and release staged tasks whose release time has arrived
        newly_released = []
        for task in self.tasks.values():
            if task.state == TaskLifecycleState.STAGED and elapsed_sec >= task.release_time_sec:
                task.transition_to(
                    TaskLifecycleState.PENDING,
                    timestamp=now_sec,
                    details=(
                        f'Deterministic dynamic release at t={elapsed_sec:.1f}s '
                        f'(scheduled: {task.release_time_sec:.1f}s)'
                    ),
                )
                self.pub_events.publish(self.event_to_msg(task.events[-1], task.task_id))
                newly_released.append(task.task_id)

        if newly_released:
            self.get_logger().info(
                f'Dynamically released {len(newly_released)} tasks at t={elapsed_sec:.1f}s: '
                f'{newly_released}'
            )

        # All tasks
        all_msg = TaskList()
        all_msg.header.stamp = now_msg
        all_msg.tasks = [self.task_to_msg(t) for t in self.tasks.values()]
        self.pub_all_tasks.publish(all_msg)

        # Available (Pending) tasks
        avail_msg = TaskList()
        avail_msg.header.stamp = now_msg
        avail_msg.tasks = [
            self.task_to_msg(t) for t in self.tasks.values()
            if t.state == TaskLifecycleState.PENDING
        ]
        self.pub_available_tasks.publish(avail_msg)

    def _handle_status_update(self, msg: TaskEventMsg) -> None:
        """Handle incoming status update transition request."""
        if msg.event_type == 'MISSION_START' or msg.new_state == 'MISSION_START':
            self._handle_mission_start()
            return

        task_id = msg.task_id
        if task_id not in self.tasks:
            self.get_logger().warn(f"Received status update for unknown task '{task_id}'")
            return

        task = self.tasks[task_id]
        try:
            target_state = TaskLifecycleState.from_str(msg.new_state)
            now_sec = self.get_clock().now().nanoseconds * 1e-9

            if task.state == target_state:
                self.get_logger().debug(
                    f"Task '{task_id}' already in state {target_state.value}, ignoring."
                )
                return

            # Robust handling of asynchronous startup races:
            # 1. If AMR reaches pickup before CBBA commit, transition:
            #    PENDING -> ASSIGNED -> IN_PROGRESS
            if (
                task.state == TaskLifecycleState.PENDING
                and target_state == TaskLifecycleState.IN_PROGRESS
            ):
                task.transition_to(
                    TaskLifecycleState.ASSIGNED,
                    timestamp=now_sec,
                    robot_id=msg.robot_id or None,
                    details='Auto-assigned upon start',
                )
                self.pub_events.publish(self.event_to_msg(task.events[-1], task_id))

            # 2. If AMR arrives at dropoff, ensure transition ASSIGNED -> IN_PROGRESS -> COMPLETED
            elif (
                task.state == TaskLifecycleState.ASSIGNED
                and target_state == TaskLifecycleState.COMPLETED
            ):
                task.transition_to(
                    TaskLifecycleState.IN_PROGRESS,
                    timestamp=now_sec,
                    robot_id=msg.robot_id or None,
                    details='Auto-progressed before dropoff completion',
                )
                self.pub_events.publish(self.event_to_msg(task.events[-1], task_id))

            # YAVI-SIH26123: Idempotent CAS Precondition Check for Autonomous Task Reclamation
            if target_state == TaskLifecycleState.PENDING and msg.details:
                reclaim_match = re.search(
                    r'from failed peer\s+(\w+)', msg.details, re.IGNORECASE
                )
                if reclaim_match:
                    expected_failed_robot = reclaim_match.group(1)
                    if task.assigned_robot_id != expected_failed_robot:
                        self.get_logger().warn(
                            f"Stale/duplicate reclamation for task '{task_id}': "
                            f"expected failed owner '{expected_failed_robot}', "
                            f"but task is currently assigned to '{task.assigned_robot_id}'. "
                            'Safely discarding.'
                        )
                        return

            # Idempotency check: redundant or late transitions should not throw errors
            if task.state == target_state:
                return
            if (
                task.state == TaskLifecycleState.IN_PROGRESS
                and target_state == TaskLifecycleState.ASSIGNED
            ):
                return
            if task.state == TaskLifecycleState.COMPLETED:
                return

            # Clear assigned_robot_id when transitioning to PENDING
            assignee = (
                None if target_state == TaskLifecycleState.PENDING
                else (msg.robot_id or None)
            )

            task.transition_to(
                target_state,
                timestamp=now_sec,
                robot_id=assignee,
                details=msg.details or 'Updated via ROS 2 interface',
            )

            # Publish the confirmed event
            latest_event = task.events[-1]
            event_msg = self.event_to_msg(latest_event, task_id)
            self.pub_events.publish(event_msg)
            self.get_logger().info(f"Task '{task_id}' transitioned to {target_state.value}")

        except InvalidTaskTransitionError as e:
            self.get_logger().error(f"Rejected transition for task '{task_id}': {e}")
        except ValueError as e:
            self.get_logger().error(f"Malformed state name for task '{task_id}': {e}")

    def _generate_unique_task_id(self) -> str:
        """Generate a unique sequential task ID not currently present in the task pool."""
        while True:
            candidate = f'T{self._task_counter:03d}'
            self._task_counter += 1
            if candidate not in self.tasks:
                return candidate

    def _handle_create_task(
        self,
        request: CreateTask.Request,
        response: CreateTask.Response,
    ) -> CreateTask.Response:
        """Handle live operator task creation via /tasks/create."""
        # 1. Resolve Task ID
        raw_id = request.task_id.strip()
        if not raw_id or raw_id.upper() in ('AUTO', 'GENERATE', 'NONE'):
            task_id = self._generate_unique_task_id()
        else:
            task_id = raw_id

        # 2. Duplicate Check
        if task_id in self.tasks:
            response.accepted = False
            response.task_id = task_id
            response.message = f"Task ID '{task_id}' already exists in pool."
            self.get_logger().warn(response.message)
            return response

        # 3. Warehouse Bounds Validation [0.0, 30.0]
        px, py = float(request.pickup_x), float(request.pickup_y)
        dx, dy = float(request.dropoff_x), float(request.dropoff_y)
        if not (0.0 <= px <= 30.0 and 0.0 <= py <= 30.0):
            response.accepted = False
            response.task_id = task_id
            response.message = (
                f'Pickup coordinate ({px:.2f}, {py:.2f}) out of warehouse bounds [0, 30].'
            )
            self.get_logger().warn(response.message)
            return response

        if not (0.0 <= dx <= 30.0 and 0.0 <= dy <= 30.0):
            response.accepted = False
            response.task_id = task_id
            response.message = (
                f'Dropoff coordinate ({dx:.2f}, {dy:.2f}) out of warehouse bounds [0, 30].'
            )
            self.get_logger().warn(response.message)
            return response

        if math.hypot(dx - px, dy - py) < 0.2:
            response.accepted = False
            response.task_id = task_id
            response.message = (
                'Pickup and dropoff coordinates must be distinct (min distance 0.20 m).'
            )
            self.get_logger().warn(response.message)
            return response

        # 4. Priority Validation
        prio_val = (
            int(request.priority)
            if request.priority in (1, 2, 3, 4)
            else int(TaskPriority.NORMAL)
        )
        priority = TaskPriority(prio_val)

        # 5. Deadline Validation
        deadline = float(request.deadline) if request.deadline > 0.0 else None

        # 6. Requested Robot Constraint Validation
        raw_robot = request.requested_robot.strip()
        if not raw_robot or raw_robot.upper() in ('AUTO', 'NONE', 'ALL'):
            requested_robot = None
        else:
            if not re.match(r'^amr_\d+$', raw_robot):
                response.accepted = False
                response.task_id = task_id
                response.message = (
                    f"Invalid requested robot format '{raw_robot}'. Expected 'amr_X'."
                )
                self.get_logger().warn(response.message)
                return response
            requested_robot = raw_robot

        # 7. Instantiate Task in PENDING state
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        task = Task(
            task_id=task_id,
            pickup=(px, py),
            dropoff=(dx, dy),
            priority=priority,
            created_at=now_sec,
            deadline=deadline,
            requested_robot=requested_robot,
        )
        self.tasks[task_id] = task

        # Publish creation event
        latest_event = task.events[-1]
        self.pub_events.publish(self.event_to_msg(latest_event, task_id))

        alloc_mode = f'DIRECT ({requested_robot})' if requested_robot else 'AUTO (CBBA)'
        response.accepted = True
        response.task_id = task_id
        response.message = (
            f"Task '{task_id}' created successfully in PENDING state (allocation: {alloc_mode})"
        )
        self.get_logger().info(response.message)

        # Trigger immediate publication of task lists
        self._publish_task_lists()
        return response

    def _handle_cancel_service(
        self,
        request: ControlTask.Request,
        response: ControlTask.Response,
    ) -> ControlTask.Response:
        """Route cancellation service call to the unified control handler."""
        request.action = 'CANCEL'
        return self._handle_control_task(request, response)

    def _handle_requeue_service(
        self,
        request: ControlTask.Request,
        response: ControlTask.Response,
    ) -> ControlTask.Response:
        """Route requeue service call to the unified control handler."""
        request.action = 'REQUEUE'
        return self._handle_control_task(request, response)

    def _handle_control_task(
        self,
        request: ControlTask.Request,
        response: ControlTask.Response,
    ) -> ControlTask.Response:
        """Handle live operator task control (CANCEL, REQUEUE)."""
        task_id = request.task_id.strip()
        action = request.action.strip().upper()

        if task_id not in self.tasks:
            response.success = False
            response.message = f"Unknown task ID '{task_id}'."
            self.get_logger().warn(response.message)
            return response

        task = self.tasks[task_id]
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if action == 'CANCEL':
            if task.is_terminal:
                response.success = False
                response.message = (
                    f"Task '{task_id}' is already in terminal state {task.state.value}."
                )
                self.get_logger().warn(response.message)
                return response

            try:
                task.transition_to(
                    TaskLifecycleState.CANCELLED,
                    timestamp=now_sec,
                    details='Cancelled by operator',
                )
                self.pub_events.publish(self.event_to_msg(task.events[-1], task_id))
                self._publish_task_lists()
                response.success = True
                response.message = f"Task '{task_id}' cancelled successfully."
                self.get_logger().info(response.message)
                return response
            except InvalidTaskTransitionError as e:
                response.success = False
                response.message = f"Cannot cancel task '{task_id}': {e}"
                self.get_logger().error(response.message)
                return response

        elif action == 'REQUEUE':
            if task.state == TaskLifecycleState.PENDING:
                response.success = True
                response.message = f"Task '{task_id}' is already PENDING."
                return response
            if task.state == TaskLifecycleState.COMPLETED:
                response.success = False
                response.message = f"Cannot requeue COMPLETED task '{task_id}'."
                self.get_logger().warn(response.message)
                return response
            if task.state == TaskLifecycleState.CANCELLED:
                response.success = False
                response.message = f"Cannot requeue CANCELLED task '{task_id}'."
                self.get_logger().warn(response.message)
                return response

            try:
                task.transition_to(
                    TaskLifecycleState.PENDING,
                    timestamp=now_sec,
                    details='Requeued by operator',
                )
                self.pub_events.publish(self.event_to_msg(task.events[-1], task_id))
                self._publish_task_lists()
                response.success = True
                response.message = f"Task '{task_id}' successfully requeued to PENDING."
                self.get_logger().info(response.message)
                return response
            except InvalidTaskTransitionError as e:
                response.success = False
                response.message = f"Cannot requeue task '{task_id}': {e}"
                self.get_logger().error(response.message)
                return response

        else:
            response.success = False
            response.message = (
                f"Unsupported control action '{action}'. Supported actions: CANCEL, REQUEUE."
            )
            return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


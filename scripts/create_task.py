#!/usr/bin/env python3
"""
CLI Tool for live operator task creation in YAVI-SIH26123 multi-AMR fleet.

Calls the /tasks/create ROS 2 service exposed by TaskManagerNode.
Supports AUTO decentralized CBBA allocation or DIRECT robot constraint.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

try:
    from amr_fleet_msgs.srv import CreateTask
    HAVE_SRV = True
except ImportError:
    HAVE_SRV = False


class TaskCreationClient(Node):
    """Client node calling /tasks/create service."""

    def __init__(self) -> None:
        super().__init__('operator_task_creator')
        self.client = self.create_client(CreateTask, '/tasks/create')

    def submit_task(
        self,
        pickup_x: float,
        pickup_y: float,
        dropoff_x: float,
        dropoff_y: float,
        priority: int = 2,
        task_id: str = '',
        deadline: float = 0.0,
        requested_robot: str = '',
        timeout_sec: float = 5.0,
    ) -> bool:
        """Call /tasks/create service synchronously."""
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                "Service '/tasks/create' is not available. Is task_manager_node running?"
            )
            return False

        req = CreateTask.Request()
        req.task_id = task_id
        req.pickup_x = float(pickup_x)
        req.pickup_y = float(pickup_y)
        req.dropoff_x = float(dropoff_x)
        req.dropoff_y = float(dropoff_y)
        req.priority = int(priority)
        req.deadline = float(deadline)
        req.requested_robot = requested_robot

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error(f"Service call timed out after {timeout_sec:.1f}s.")
            return False

        resp = future.result()
        if resp is None:
            self.get_logger().error('Received null response from /tasks/create.')
            return False

        if resp.accepted:
            print('\n========================================')
            print('  TASK CREATION ACCEPTED')
            print('========================================')
            print(f'  Task ID:          {resp.task_id}')
            print(f'  Pickup:           ({pickup_x:.2f}, {pickup_y:.2f})')
            print(f'  Dropoff:          ({dropoff_x:.2f}, {dropoff_y:.2f})')
            prio_map = {1: 'LOW', 2: 'NORMAL', 3: 'HIGH', 4: 'CRITICAL'}
            print(f'  Priority:         {prio_map.get(priority, str(priority))}')
            alloc = f'DIRECT ({requested_robot})' if requested_robot else 'AUTO (CBBA Consensus)'
            print(f'  Allocation:       {alloc}')
            if deadline > 0.0:
                print(f'  Deadline:         {deadline:.1f} s')
            print(f'  Message:          {resp.message}')
            print('========================================\n')
            return True
        else:
            print('\n========================================')
            print('  TASK CREATION REJECTED')
            print('========================================')
            print(f'  Attempted ID:     {resp.task_id or task_id or "(auto)"}')
            print(f'  Reason:           {resp.message}')
            print('========================================\n')
            return False


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Create a live task for the YAVI-SIH26123 AMR fleet via /tasks/create.'
    )
    parser.add_argument(
        '--pickup-x', type=float, required=True, help='Pickup X coordinate [0.0 - 30.0]'
    )
    parser.add_argument(
        '--pickup-y', type=float, required=True, help='Pickup Y coordinate [0.0 - 30.0]'
    )
    parser.add_argument(
        '--dropoff-x', type=float, required=True, help='Dropoff X coordinate [0.0 - 30.0]'
    )
    parser.add_argument(
        '--dropoff-y', type=float, required=True, help='Dropoff Y coordinate [0.0 - 30.0]'
    )
    parser.add_argument(
        '--priority',
        type=str,
        default='NORMAL',
        help='Priority level: LOW, NORMAL, HIGH, CRITICAL, or 1-4 (default: NORMAL)',
    )
    parser.add_argument(
        '--task-id',
        type=str,
        default='',
        help="Custom Task ID (optional; defaults to auto-generated sequential 'T###')",
    )
    parser.add_argument(
        '--robot',
        '--requested-robot',
        dest='requested_robot',
        type=str,
        default='',
        help="Target robot ID for DIRECT assignment (e.g. 'amr_4'). Omit for AUTO CBBA auction.",
    )
    parser.add_argument(
        '--deadline',
        type=float,
        default=0.0,
        help='Task completion deadline in simulation seconds (optional)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='Service response timeout in seconds (default: 5.0)',
    )

    args = parser.parse_args()

    # Parse priority
    prio_str = args.priority.strip().upper()
    prio_map = {'LOW': 1, 'NORMAL': 2, 'HIGH': 3, 'CRITICAL': 4}
    if prio_str in prio_map:
        priority_val = prio_map[prio_str]
    else:
        try:
            priority_val = int(args.priority)
        except ValueError:
            priority_val = 2

    if not HAVE_SRV:
        print(
            "ERROR: 'amr_fleet_msgs.srv.CreateTask' not found. "
            'Please source the ROS 2 workspace (install/setup.bash).'
        )
        sys.exit(1)

    rclpy.init()
    node = TaskCreationClient()
    try:
        success = node.submit_task(
            pickup_x=args.pickup_x,
            pickup_y=args.pickup_y,
            dropoff_x=args.dropoff_x,
            dropoff_y=args.dropoff_y,
            priority=priority_val,
            task_id=args.task_id,
            deadline=args.deadline,
            requested_robot=args.requested_robot,
            timeout_sec=args.timeout,
        )
        sys.exit(0 if success else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


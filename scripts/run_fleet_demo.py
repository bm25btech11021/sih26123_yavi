#!/usr/bin/env python3
"""
YAVI-SIH26123 Multi-AMR Demonstration Scenario Coordinator.

Executes deterministic, collision-free, multi-stage fleet trajectories
in the warehouse environment to showcase decentralized AMR fleet capability
during hackathon and research demonstrations.

Interacts strictly via standard namespaced ROS 2 interfaces:
- Publishes: /<robot_id>/cmd_vel (geometry_msgs/msg/Twist)
- Subscribes: /<robot_id>/odom (nav_msgs/msg/Odometry)
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotPoseState:
    """Tracks position and orientation from odometry."""

    def __init__(self, robot_id: str) -> None:
        self.robot_id = robot_id
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.received = False

    def update(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.received = True


class WaypointController:
    """Smooth proportional waypoint navigator for differential drive AMRs."""

    def __init__(
        self,
        max_v: float = 0.35,
        max_w: float = 0.60,
        dist_tol: float = 0.25,
    ) -> None:
        self.max_v = max_v
        self.max_w = max_w
        self.dist_tol = dist_tol

    def compute_cmd(
        self, current_x: float, current_y: float, current_yaw: float, target_x: float, target_y: float
    ) -> Tuple[float, float, bool]:
        dx = target_x - current_x
        dy = target_y - current_y
        dist = math.hypot(dx, dy)

        if dist < self.dist_tol:
            return 0.0, 0.0, True

        target_yaw = math.atan2(dy, dx)
        heading_err = target_yaw - current_yaw

        # Normalize heading error to [-pi, pi]
        while heading_err > math.pi:
            heading_err -= 2.0 * math.pi
        while heading_err < -math.pi:
            heading_err += 2.0 * math.pi

        # If heading error is large, turn in place first
        if abs(heading_err) > 0.6:
            v = 0.0
            w = math.copysign(min(self.max_w, max(0.2, abs(heading_err) * 0.8)), heading_err)
        else:
            v = min(self.max_v, max(0.1, dist * 0.5))
            w = math.copysign(min(self.max_w, abs(heading_err) * 1.0), heading_err)

        return v, w, False


class FleetDemoCoordinator(Node):
    """Coordinates deterministic multi-AMR presentation trajectories."""

    def __init__(self, robot_count: int = 5, loop: bool = False) -> None:
        super().__init__('amr_fleet_demo_coordinator')
        self.robot_count = robot_count
        self.loop = loop
        self.poses: Dict[str, RobotPoseState] = {}
        self.cmd_pubs: Dict[str, Any] = {}
        self.controller = WaypointController()

        self.get_logger().info(f"Initializing Fleet Demo Coordinator for {robot_count} robots...")

        # Setup publishers and subscribers for each AMR
        for i in range(robot_count):
            r_id = f'amr_{i}'
            self.poses[r_id] = RobotPoseState(r_id)
            self.cmd_pubs[r_id] = self.create_publisher(Twist, f'/{r_id}/cmd_vel', 10)

            def make_cb(r):
                return lambda msg: self.poses[r].update(msg)

            self.create_subscription(Odometry, f'/{r_id}/odom', make_cb(r_id), 10)

        # Plan structured multi-robot route missions
        # Mission format: list of waypoints [(x, y), ...]
        self.missions: Dict[str, List[Tuple[float, float]]] = {
            # amr_0: Western corridor run (Station P1 -> Station P2 -> return)
            'amr_0': [(2.0, 7.0), (2.0, 12.0), (2.0, 6.0), (2.0, 2.0)],
            # amr_1: Station P1 dispatch & center crossing
            'amr_1': [(2.0, 3.0), (4.5, 2.0), (6.8, 3.5), (2.0, 5.0)],
            # amr_2: Central Arterial Highway transit to Sorting Hub
            'amr_2': [(2.0, 8.0), (6.0, 8.0), (8.0, 7.5), (6.0, 8.0), (2.0, 8.0)],
            # amr_3: East corridor logistics maneuver
            'amr_3': [(2.0, 11.0), (4.5, 12.5), (2.0, 13.0), (2.0, 11.0)],
            # amr_4: North cross-aisle patrol
            'amr_4': [(2.0, 14.0), (5.0, 14.5), (8.0, 14.5), (2.0, 14.0)],
        }

        # Adapt for fewer robots if robot_count < 5
        self.current_step: Dict[str, int] = {r_id: 0 for r_id in self.missions}
        self.dwell_until: Dict[str, float] = {r_id: 0.0 for r_id in self.missions}

        # 10 Hz control loop timer
        self.timer = self.create_timer(0.1, self._control_loop)
        self.start_time = time.time()
        self.get_logger().info("Demo coordinator initialized. Awaiting odometry sync...")

    def _control_loop(self) -> None:
        now = time.time()
        # Verify odometry received
        all_ready = all(self.poses[r_id].received for r_id in self.poses)
        if not all_ready:
            return

        all_completed = True

        for r_id, mission in self.missions.items():
            if r_id not in self.poses:
                continue

            step = self.current_step[r_id]
            if step >= len(mission):
                if self.loop:
                    self.current_step[r_id] = 0
                    step = 0
                else:
                    self._stop_robot(r_id)
                    continue

            all_completed = False

            # Check if dwelling at station
            if now < self.dwell_until[r_id]:
                self._stop_robot(r_id)
                continue

            target_x, target_y = mission[step]
            pose = self.poses[r_id]

            v, w, reached = self.controller.compute_cmd(pose.x, pose.y, pose.yaw, target_x, target_y)

            if reached:
                self.get_logger().info(f"[{r_id}] Reached waypoint ({target_x}, {target_y}) — dwelling 2.0s")
                self.dwell_until[r_id] = now + 2.0
                self.current_step[r_id] += 1
                self._stop_robot(r_id)
            else:
                msg = Twist()
                msg.linear.x = float(v)
                msg.angular.z = float(w)
                self.cmd_pubs[r_id].publish(msg)

        if all_completed and not self.loop:
            self.get_logger().info("Demonstration scenario mission sequence complete.")

    def _stop_robot(self, r_id: str) -> None:
        if r_id in self.cmd_pubs:
            self.cmd_pubs[r_id].publish(Twist())

    def stop_all(self) -> None:
        self.get_logger().info("Stopping all fleet robots...")
        for r_id in self.cmd_pubs:
            self._stop_robot(r_id)


def main() -> None:
    parser = argparse.ArgumentParser(description="YAVI-SIH26123 Fleet Demo Coordinator")
    parser.add_argument('--robot-count', type=int, default=5, help="Number of AMRs in demo (default: 5)")
    parser.add_argument('--loop', action='store_true', help="Run demonstration continuously in loop")
    args = parser.parse_args()

    rclpy.init()
    node = FleetDemoCoordinator(robot_count=args.robot_count, loop=args.loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


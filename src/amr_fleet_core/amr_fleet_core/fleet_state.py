"""Fleet state discovery and metadata abstraction for multi-AMR coordination."""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import yaml


@dataclass
class RobotInfo:
    """Metadata and interface endpoints for a single AMR in the fleet."""

    robot_id: str
    namespace: str
    pose_topic: str
    cmd_vel_topic: str
    scan_topic: str
    status: str = 'ACTIVE'
    initial_pose: Optional[Tuple[float, float, float, float]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert robot metadata to dictionary representation."""
        return {
            'robot_id': self.robot_id,
            'namespace': self.namespace,
            'pose_topic': self.pose_topic,
            'cmd_vel_topic': self.cmd_vel_topic,
            'scan_topic': self.scan_topic,
            'status': self.status,
            'initial_pose': list(self.initial_pose) if self.initial_pose else None,
        }


class FleetState:
    """Lightweight registry of fleet robots, endpoints, and statuses."""

    def __init__(self, robots: Optional[Dict[str, RobotInfo]] = None) -> None:
        """Initialize fleet registry with optional dictionary of robots."""
        self._robots: Dict[str, RobotInfo] = robots if robots is not None else {}

    def add_robot(self, robot: RobotInfo) -> None:
        """Register a robot in the fleet state."""
        self._robots[robot.robot_id] = robot

    def get_robot(self, robot_id: str) -> Optional[RobotInfo]:
        """Retrieve robot metadata by ID, or None if not found."""
        return self._robots.get(robot_id)

    def list_robots(self) -> List[RobotInfo]:
        """Return a sorted list of all registered robots."""
        return [self._robots[k] for k in sorted(self._robots.keys())]

    def active_robot_ids(self) -> List[str]:
        """Return list of IDs for all robots with ACTIVE status."""
        return [
            k for k, v in sorted(self._robots.items())
            if v.status == 'ACTIVE'
        ]

    def count(self) -> int:
        """Return total number of registered robots."""
        return len(self._robots)

    def contains(self, robot_id: str) -> bool:
        """Check whether a robot ID is registered in the fleet."""
        return robot_id in self._robots

    @classmethod
    def from_yaml(cls, yaml_path: str, max_count: Optional[int] = None) -> 'FleetState':
        """Construct FleetState from a YAML fleet configuration file."""
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        fleet_data = data.get('fleet', {})
        robot_list = fleet_data.get('robots', [])

        if max_count is not None and max_count > 0:
            robot_list = robot_list[:max_count]

        state = cls()
        for entry in robot_list:
            r_id = entry['id']
            ns = f'/{r_id}'
            pose = (
                float(entry.get('x', 0.0)),
                float(entry.get('y', 0.0)),
                float(entry.get('z', 0.15)),
                float(entry.get('yaw', 0.0)),
            )
            info = RobotInfo(
                robot_id=r_id,
                namespace=ns,
                pose_topic=f'{ns}/odom',
                cmd_vel_topic=f'{ns}/cmd_vel',
                scan_topic=f'{ns}/scan',
                status='ACTIVE',
                initial_pose=pose,
            )
            state.add_robot(info)

        return state

    @classmethod
    def from_topics(cls, topic_names: List[str]) -> 'FleetState':
        """Discover active robots from a list of live ROS 2 topic names."""
        discovered_namespaces: Dict[str, Dict[str, str]] = {}

        for topic in topic_names:
            parts = [p for p in topic.strip('/').split('/') if p]
            if len(parts) >= 2 and parts[0].startswith('amr_'):
                r_id = parts[0]
                sub_topic = parts[1]
                if r_id not in discovered_namespaces:
                    discovered_namespaces[r_id] = {}
                discovered_namespaces[r_id][sub_topic] = topic

        state = cls()
        for r_id, topics in discovered_namespaces.items():
            ns = f'/{r_id}'
            info = RobotInfo(
                robot_id=r_id,
                namespace=ns,
                pose_topic=topics.get('odom', f'{ns}/odom'),
                cmd_vel_topic=topics.get('cmd_vel', f'{ns}/cmd_vel'),
                scan_topic=topics.get('scan', f'{ns}/scan'),
                status='ACTIVE',
            )
            state.add_robot(info)

        return state


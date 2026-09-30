"""Publish 3D RViz markers for warehouse obstacles, stations, and boundaries."""

import os
from typing import Any, List

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import yaml


class WarehouseVisualizerNode(Node):
    """Publishes static 3D warehouse geometry to RViz on /warehouse_markers."""

    def __init__(self) -> None:
        super().__init__('warehouse_visualizer')

        self.declare_parameter('config_file', '')
        cfg_path = self.get_parameter('config_file').value

        # Default 16x16 warehouse topology
        self.width = 16.0
        self.height = 16.0
        self.pickups: List[Any] = [(2.0, 2.0), (2.0, 13.0), (13.0, 2.0), (13.0, 13.0)]
        self.dropoffs: List[Any] = [(7.0, 7.0), (7.0, 8.0), (8.0, 7.0), (8.0, 8.0)]
        self.charging_pads: List[Any] = []
        self.obstacles: List[Any] = [
            (4, 4), (4, 5), (4, 6), (4, 9), (4, 10), (4, 11),
            (11, 4), (11, 5), (11, 6), (11, 9), (11, 10), (11, 11),
        ]

        # Load from file if provided
        if not cfg_path:
            candidate_paths = [
                os.path.join(os.getcwd(), 'config', 'maps', 'warehouse_m9_v2.yaml'),
                os.path.join(os.getcwd(), 'config', 'maps', 'warehouse_grid_small.yaml'),
                
                
            ]
            for c in candidate_paths:
                if os.path.isfile(c):
                    cfg_path = c
                    break

        if cfg_path and os.path.isfile(cfg_path):
            try:
                with open(cfg_path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    dims = data.get('dimensions', {})
                    self.width = float(dims.get('x', dims.get('width', self.width)))
                    self.height = float(dims.get('y', dims.get('height', self.height)))
                    st = data.get('stations', {})
                    if 'pickups' in st:
                        raw_pickups = st['pickups']
                        parsed_pickups = []
                        for p in raw_pickups:
                            if isinstance(p, dict):
                                coords = p.get('coords', [0.0, 0.0])
                                parsed_pickups.append((float(coords[0]), float(coords[1])))
                            elif isinstance(p, (list, tuple)):
                                parsed_pickups.append((float(p[0]), float(p[1])))
                        self.pickups = parsed_pickups
                    if 'dropoffs' in st:
                        raw_dropoffs = st['dropoffs']
                        parsed_dropoffs = []
                        for d in raw_dropoffs:
                            if isinstance(d, dict):
                                coords = d.get('coords', [0.0, 0.0])
                                parsed_dropoffs.append((float(coords[0]), float(coords[1])))
                            elif isinstance(d, (list, tuple)):
                                parsed_dropoffs.append((float(d[0]), float(d[1])))
                        self.dropoffs = parsed_dropoffs
                    if 'charging_pads' in st:
                        raw_charging = st['charging_pads']
                        parsed_charging = []
                        for cp in raw_charging:
                            if isinstance(cp, dict):
                                coords = cp.get('coords', [0.0, 0.0])
                                parsed_charging.append((float(coords[0]), float(coords[1])))
                            elif isinstance(cp, (list, tuple)):
                                parsed_charging.append((float(cp[0]), float(cp[1])))
                        self.charging_pads = parsed_charging
                    if 'obstacles' in data:
                        raw_obs = data['obstacles']
                        parsed_obs: List[Any] = []
                        for o in raw_obs:
                            if isinstance(o, dict):
                                parsed_obs.append(o)
                            elif isinstance(o, (list, tuple)):
                                parsed_obs.append((float(o[0]), float(o[1])))
                        self.obstacles = parsed_obs
            except Exception as e:
                self.get_logger().warn(f'Failed to parse map YAML: {e}')

        latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, '/warehouse_markers', latched_qos
        )

        self.timer = self.create_timer(1.0, self._publish_markers)
        self.get_logger().info(
            f'WarehouseVisualizerNode started with config: {cfg_path}. '
            f'Publishing to /warehouse_markers.'
        )

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        m_id = 0

        # 1. Perimeter Walls (4 bounding walls)
        wall_marker = Marker()
        wall_marker.header.frame_id = 'map'
        wall_marker.header.stamp = now
        wall_marker.ns = 'walls'
        wall_marker.id = m_id
        m_id += 1
        wall_marker.type = Marker.LINE_LIST
        wall_marker.action = Marker.ADD
        wall_marker.scale.x = 0.15
        wall_marker.color.r = 0.5
        wall_marker.color.g = 0.5
        wall_marker.color.b = 0.6
        wall_marker.color.a = 1.0

        corners = [
            (0.0, 0.0), (float(self.width), 0.0),
            (float(self.width), float(self.height)),
            (0.0, float(self.height)),
        ]
        for i in range(4):
            c1 = corners[i]
            c2 = corners[(i + 1) % 4]
            p1 = Point(x=c1[0], y=c1[1], z=0.0)
            p2 = Point(x=c2[0], y=c2[1], z=0.0)
            p3 = Point(x=c1[0], y=c1[1], z=1.5)
            p4 = Point(x=c2[0], y=c2[1], z=1.5)
            # Bottom rail
            wall_marker.points.extend([p1, p2])
            # Top rail
            wall_marker.points.extend([p3, p4])
            # Vertical post
            wall_marker.points.extend([p1, p3])

        marker_array.markers.append(wall_marker)

        # 2. Shelving Racks (Obstacles) - 3D Industrial Racks
        for obst in self.obstacles:
            rack = Marker()
            rack.header.frame_id = 'map'
            rack.header.stamp = now
            rack.ns = 'shelves'
            rack.id = m_id
            m_id += 1
            rack.type = Marker.CUBE
            rack.action = Marker.ADD
            if isinstance(obst, dict):
                center = obst.get('center', [0.0, 0.0])
                size = obst.get('size', [1.2, 3.0])
                rack.pose.position.x = float(center[0])
                rack.pose.position.y = float(center[1])
                rack.pose.position.z = 1.0
                rack.scale.x = float(size[0])
                rack.scale.y = float(size[1])
                rack.scale.z = 2.0
            else:
                ox, oy = obst[0], obst[1]
                rack.pose.position.x = ox + 0.5
                rack.pose.position.y = oy + 0.5
                rack.pose.position.z = 0.9
                rack.scale.x = 0.85
                rack.scale.y = 0.85
                rack.scale.z = 1.8
            rack.color.r = 0.25
            rack.color.g = 0.30
            rack.color.b = 0.38
            rack.color.a = 0.95
            marker_array.markers.append(rack)

        # 3. Pickup Stations (Green floor zones + floating text)
        for idx, (px, py) in enumerate(self.pickups):
            pad = Marker()
            pad.header.frame_id = 'map'
            pad.header.stamp = now
            pad.ns = 'pickup_pads'
            pad.id = m_id
            m_id += 1
            pad.type = Marker.CUBE
            pad.action = Marker.ADD
            pad.pose.position.x = float(px)
            pad.pose.position.y = float(py)
            pad.pose.position.z = 0.02
            pad.scale.x = 1.6
            pad.scale.y = 1.6
            pad.scale.z = 0.04
            pad.color.r = 0.0
            pad.color.g = 0.85
            pad.color.b = 0.35
            pad.color.a = 0.6
            marker_array.markers.append(pad)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'pickup_text'
            text.id = m_id
            m_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(px)
            text.pose.position.y = float(py)
            text.pose.position.z = 1.2
            text.scale.z = 0.4
            text.color.r = 0.1
            text.color.g = 1.0
            text.color.b = 0.4
            text.color.a = 0.9
            text.text = f'PICKUP P{idx+1}'
            marker_array.markers.append(text)

        # 4. Dropoff Hubs (Blue zones + floating text)
        for idx, (dx, dy) in enumerate(self.dropoffs):
            drop_pad = Marker()
            drop_pad.header.frame_id = 'map'
            drop_pad.header.stamp = now
            drop_pad.ns = 'dropoff_pad'
            drop_pad.id = m_id
            m_id += 1
            drop_pad.type = Marker.CUBE
            drop_pad.action = Marker.ADD
            drop_pad.pose.position.x = float(dx)
            drop_pad.pose.position.y = float(dy)
            drop_pad.pose.position.z = 0.02
            drop_pad.scale.x = 2.0
            drop_pad.scale.y = 2.0
            drop_pad.scale.z = 0.04
            drop_pad.color.r = 0.1
            drop_pad.color.g = 0.55
            drop_pad.color.b = 1.0
            drop_pad.color.a = 0.6
            marker_array.markers.append(drop_pad)

            drop_text = Marker()
            drop_text.header.frame_id = 'map'
            drop_text.header.stamp = now
            drop_text.ns = 'dropoff_text'
            drop_text.id = m_id
            m_id += 1
            drop_text.type = Marker.TEXT_VIEW_FACING
            drop_text.action = Marker.ADD
            drop_text.pose.position.x = float(dx)
            drop_text.pose.position.y = float(dy)
            drop_text.pose.position.z = 1.3
            drop_text.scale.z = 0.5
            drop_text.color.r = 0.3
            drop_text.color.g = 0.75
            drop_text.color.b = 1.0
            drop_text.color.a = 0.95
            drop_text.text = f'DROPOFF D{idx+1}'
            marker_array.markers.append(drop_text)

        # 5. Charging Pads (Amber zones + floating text)
        for idx, (cx, cy) in enumerate(self.charging_pads):
            chg_pad = Marker()
            chg_pad.header.frame_id = 'map'
            chg_pad.header.stamp = now
            chg_pad.ns = 'charge_pad'
            chg_pad.id = m_id
            m_id += 1
            chg_pad.type = Marker.CUBE
            chg_pad.action = Marker.ADD
            chg_pad.pose.position.x = float(cx)
            chg_pad.pose.position.y = float(cy)
            chg_pad.pose.position.z = 0.02
            chg_pad.scale.x = 1.2
            chg_pad.scale.y = 1.2
            chg_pad.scale.z = 0.04
            chg_pad.color.r = 1.0
            chg_pad.color.g = 0.65
            chg_pad.color.b = 0.0
            chg_pad.color.a = 0.6
            marker_array.markers.append(chg_pad)

            chg_text = Marker()
            chg_text.header.frame_id = 'map'
            chg_text.header.stamp = now
            chg_text.ns = 'charge_text'
            chg_text.id = m_id
            m_id += 1
            chg_text.type = Marker.TEXT_VIEW_FACING
            chg_text.action = Marker.ADD
            chg_text.pose.position.x = float(cx)
            chg_text.pose.position.y = float(cy)
            chg_text.pose.position.z = 1.1
            chg_text.scale.z = 0.35
            chg_text.color.r = 1.0
            chg_text.color.g = 0.8
            chg_text.color.b = 0.2
            chg_text.color.a = 0.9
            chg_text.text = f'CHG {idx+1}'
            marker_array.markers.append(chg_text)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseVisualizerNode()
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


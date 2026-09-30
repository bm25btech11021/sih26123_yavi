"""Parameterized launch description for multi-AMR fleet simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_amr_bringup = get_package_share_directory('amr_fleet_bringup')

    robot_count_str = context.perform_substitution(LaunchConfiguration('robot_count'))
    try:
        robot_count = int(robot_count_str)
    except ValueError:
        robot_count = 2

    fleet_config_arg = context.perform_substitution(LaunchConfiguration('fleet_config'))
    world_name = context.perform_substitution(LaunchConfiguration('world'))
    headless_str = context.perform_substitution(LaunchConfiguration('headless')).lower()
    headless = headless_str in ('true', '1')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resolve world path
    clean_world_name = world_name[:-4] if world_name.endswith('.sdf') else world_name
    world_path = os.path.join(pkg_amr_bringup, 'worlds', f'{clean_world_name}.sdf')
    gz_args = f'-r -s {world_path}' if headless else f'-r {world_path}'

    # Resolve robot configurations
    current_dir = os.path.dirname(os.path.abspath(__file__))
    cur = current_dir
    workspace_root = None
    for _ in range(6):
        if os.path.isdir(os.path.join(cur, 'config', 'robots')):
            workspace_root = cur
            break
        cur = os.path.dirname(cur)
    if not workspace_root:
        if os.path.isdir(os.path.join(os.getcwd(), 'config', 'robots')):
            workspace_root = os.getcwd()
        else:
            workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    map_config_arg = context.perform_substitution(LaunchConfiguration('map_config_file'))
    config_dir = os.path.join(workspace_root, 'config', 'robots')
    maps_dir = os.path.join(workspace_root, 'config', 'maps')

    # Resolve map config file
    tag = clean_world_name.replace('warehouse_', '')
    cand_map_1 = os.path.join(maps_dir, f'{clean_world_name}.yaml')
    cand_map_2 = os.path.join(maps_dir, f'warehouse_{tag}.yaml')
    cand_map_3 = os.path.join(maps_dir, 'warehouse_grid_small.yaml')

    resolved_map_file = ''
    if map_config_arg and os.path.isfile(map_config_arg):
        resolved_map_file = map_config_arg
    elif os.path.isfile(cand_map_1):
        resolved_map_file = cand_map_1
    elif os.path.isfile(cand_map_2):
        resolved_map_file = cand_map_2
    elif os.path.isfile(cand_map_3):
        resolved_map_file = cand_map_3

    config_path = None
    if fleet_config_arg and os.path.isfile(fleet_config_arg):
        config_path = fleet_config_arg
    else:
        cand_world_1 = os.path.join(
            config_dir, f'fleet_{robot_count}_robots_{tag}.yaml'
        )
        cand_world_2 = os.path.join(
            config_dir, f'fleet_{robot_count}_robots_{clean_world_name}.yaml'
        )
        candidate = os.path.join(
            config_dir, f'fleet_{robot_count}_robots.yaml'
        )
        default_candidate = os.path.join(config_dir, 'fleet_default.yaml')
        if os.path.isfile(cand_world_1):
            config_path = cand_world_1
        elif os.path.isfile(cand_world_2):
            config_path = cand_world_2
        elif os.path.isfile(candidate):
            config_path = candidate
        elif os.path.isfile(default_candidate):
            config_path = default_candidate

    robot_configs = []
    if config_path and os.path.isfile(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        robot_configs = data.get('fleet', {}).get('robots', [])

    # If configuration has fewer robots than requested, generate poses
    is_32m = 'm9' in clean_world_name or '32' in clean_world_name
    while len(robot_configs) < robot_count:
        idx = len(robot_configs)
        col = idx // 5
        row = idx % 5
        if is_32m:
            x_coord = 2.5 if col == 0 else 16.0
            y_coord = 5.0 + row * 5.0
        else:
            x_coord = 2.0 if col == 0 else 8.0
            y_coord = 2.0 + row * 3.0
        robot_configs.append({
            'id': f'amr_{idx}',
            'x': x_coord,
            'y': y_coord,
            'z': 0.15,
            'yaw': 0.0,
        })

    # Slice to exact robot_count
    robot_configs = robot_configs[:robot_count]

    log_info = LogInfo(
        msg=(
            f'[FLEET] Spawning {robot_count} robots into world "{world_name}" '
            f'(headless={headless})'
        )
    )

    # Launch Gazebo simulation server
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Global Clock bridge (GZ -> ROS)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Parameterized spawn for each robot
    spawn_actions = []
    spawn_launch_path = os.path.join(pkg_amr_bringup, 'launch', 'spawn_robot.launch.py')

    for r in robot_configs:
        spawn_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch_path),
                launch_arguments={
                    'world': clean_world_name,
                    'robot_name': str(r['id']),
                    'x': str(r['x']),
                    'y': str(r['y']),
                    'z': str(r.get('z', 0.15)),
                    'yaw': str(r.get('yaw', 0.0)),
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        )
        spawn_actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{r["id"]}_map_to_odom',
                arguments=[
                    '--x', str(r['x']),
                    '--y', str(r['y']),
                    '--z', '0.0',
                    '--yaw', str(r.get('yaw', 0.0)),
                    '--pitch', '0.0',
                    '--roll', '0.0',
                    '--frame-id', 'map',
                    '--child-frame-id', f'{r["id"]}/odom',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )

    # Delay robot spawning slightly so Gazebo Harmonic finishes initializing
    # its world and transport service
    delayed_spawns = TimerAction(
        period=2.0,
        actions=spawn_actions,
    )

    # Optional RViz
    rviz_config = os.path.join(pkg_amr_bringup, 'rviz', 'fleet_default.rviz')
    if not os.path.isfile(rviz_config):
        rviz_config = os.path.join(pkg_amr_bringup, 'rviz', 'single_amr.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    warehouse_viz_node = Node(
        package='amr_fleet_bringup',
        executable='warehouse_visualizer',
        name='warehouse_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': resolved_map_file,
        }],
    )

    return [log_info, gz_sim, clock_bridge, warehouse_viz_node, delayed_spawns, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_count',
            default_value='10',
            description='Number of AMRs to spawn in fleet simulation (1 to 10)',
        ),
        DeclareLaunchArgument(
            'fleet_config',
            default_value='',
            description='Path to custom YAML fleet configuration file (optional)',
        ),
        DeclareLaunchArgument(
            'map_config_file',
            default_value='',
            description='Path to map YAML configuration file (optional)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_small',
            description='Simulation world name (without .sdf extension)',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo Harmonic in headless server mode',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock for all nodes',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for fleet visualization',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


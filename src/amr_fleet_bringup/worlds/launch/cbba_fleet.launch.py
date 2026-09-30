"""
Launch decentralized CBBA task allocation fleet across multiple AMRs.

Spawns Gazebo simulation, spawns N robots, starts the M3 Task Manager,
and instantiates independent decentralized CBBA nodes for each robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Dynamically configure and instantiate CBBA nodes per robot."""
    pkg_amr_bringup = get_package_share_directory('amr_fleet_bringup')
    fleet_launch_path = os.path.join(pkg_amr_bringup, 'launch', 'fleet.launch.py')

    robot_count_str = context.perform_substitution(LaunchConfiguration('robot_count'))
    try:
        robot_count = int(robot_count_str)
    except ValueError:
        robot_count = 5

    max_bundle_size_str = context.perform_substitution(
        LaunchConfiguration('max_bundle_size')
    )
    try:
        max_bundle_size = int(max_bundle_size_str)
    except ValueError:
        max_bundle_size = 4

    workload_file = context.perform_substitution(
        LaunchConfiguration('workload_file')
    )

    actions = []

    # 1. Fleet Simulation (Gazebo + Robot Spawners)
    launch_sim = context.perform_substitution(
        LaunchConfiguration('launch_simulation')
    ).lower() in ('true', '1')

    if launch_sim:
        fleet_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fleet_launch_path),
            launch_arguments={
                'robot_count': str(robot_count),
                'headless': LaunchConfiguration('headless'),
                'rviz': LaunchConfiguration('rviz'),
                'world': LaunchConfiguration('world'),
                'use_sim_time': 'true',
            }.items(),
        )
        actions.append(fleet_launch)

    # 2. Task Manager Node
    # If workload_file not provided, use default medium priority workload
    if not workload_file:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
        candidate = os.path.join(
            workspace_root, 'config', 'workloads', 'workload_medium_priority.yaml'
        )
        if os.path.isfile(candidate):
            workload_file = candidate

    task_mgr_node = Node(
        package='amr_fleet_core',
        executable='task_manager',
        name='amr_task_manager',
        output='screen',
        parameters=[{
            'workload_file': workload_file,
            'publish_rate': 2.0,
        }],
    )
    actions.append(task_mgr_node)

    # 3. Decentralized CBBA Node per AMR
    actions.append(
        LogInfo(
            msg=f'[CBBA Bringup] Spawning {robot_count} CBBA nodes (cap: {max_bundle_size})'
        )
    )

    for i in range(robot_count):
        robot_id = f'amr_{i}'
        cbba_node = Node(
            package='amr_fleet_core',
            executable='cbba_node',
            name='cbba_node',
            namespace=robot_id,
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'max_bundle_size': max_bundle_size,
                'consensus_rate': 5.0,
                'stable_rounds_for_convergence': 5,
            }],
        )
        actions.append(cbba_node)

    return actions


def generate_launch_description():
    """Generate launch description with arguments and opaque function."""
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_count',
            default_value='5',
            description='Number of AMRs participating in CBBA allocation (default: 5)',
        ),
        DeclareLaunchArgument(
            'max_bundle_size',
            default_value='4',
            description='Maximum tasks per robot bundle (default: 4)',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (default: false)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 alongside Gazebo (default: false)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_small',
            description='Warehouse world file name without extension',
        ),
        DeclareLaunchArgument(
            'workload_file',
            default_value='',
            description='Path to workload YAML file',
        ),
        DeclareLaunchArgument(
            'launch_simulation',
            default_value='true',
            description='Whether to launch Gazebo simulation (default: true)',
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


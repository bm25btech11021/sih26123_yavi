"""Launch description for single AMR simulation in Gazebo Harmonic with ROS 2 Jazzy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_amr_bringup = get_package_share_directory('amr_fleet_bringup')

    headless_str = context.perform_substitution(LaunchConfiguration('headless')).lower()
    headless = headless_str in ('true', '1')
    world_name = context.perform_substitution(LaunchConfiguration('world'))
    world_path = os.path.join(pkg_amr_bringup, 'worlds', f'{world_name}.sdf')

    gz_args = f'-r -s {world_path}' if headless else f'-r {world_path}'

    # Gazebo simulation server
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

    # Robot spawner
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_bringup, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # Optional RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_amr_bringup, 'rviz', 'single_amr.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return [
        gz_sim,
        clock_bridge,
        spawn_robot,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo Harmonic in headless server mode',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_small',
            description='Simulation world name (without .sdf extension)',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='amr_0',
            description='Robot namespace and model name',
        ),
        DeclareLaunchArgument('x', default_value='2.0', description='Initial X coordinate'),
        DeclareLaunchArgument('y', default_value='2.0', description='Initial Y coordinate'),
        DeclareLaunchArgument('z', default_value='0.15', description='Initial Z coordinate'),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial Yaw angle in radians',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock for all nodes',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for visualization',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


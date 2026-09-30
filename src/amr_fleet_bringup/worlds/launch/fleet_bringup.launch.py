"""Parameterized fleet simulation launcher (alias for fleet.launch.py)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_fleet_bringup')
    fleet_launch_path = os.path.join(pkg_amr_bringup, 'launch', 'fleet.launch.py')

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_count',
            default_value='2',
            description='Number of AMRs to spawn in fleet simulation',
        ),
        DeclareLaunchArgument(
            'fleet_config',
            default_value='',
            description='Path to custom YAML fleet configuration file',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo Harmonic server in headless mode',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_small',
            description='Simulation world name',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for fleet visualization',
        ),
    ]

    include_fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fleet_launch_path),
        launch_arguments={
            'robot_count': LaunchConfiguration('robot_count'),
            'fleet_config': LaunchConfiguration('fleet_config'),
            'headless': LaunchConfiguration('headless'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [include_fleet])


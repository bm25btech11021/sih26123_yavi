# Copyright 2026 Raghav Gangrade
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch description for previewing AMR description in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_amr_description = get_package_share_directory('amr_fleet_description')

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_name',
            default_value='amr_0',
            description='Name / namespace of the robot',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='amr_0/',
            description='Prefix for robot link and joint names',
        ),
    ]

    robot_name = LaunchConfiguration('robot_name')
    prefix = LaunchConfiguration('prefix')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('amr_fleet_description'), 'urdf', 'amr.urdf.xacro']
            ),
            ' ',
            'robot_name:=', robot_name,
            ' ',
            'prefix:=', prefix,
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=robot_name,
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_amr_description, 'rviz', 'view_robot.rviz')],
        output='screen',
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz_node,
        ]
    )


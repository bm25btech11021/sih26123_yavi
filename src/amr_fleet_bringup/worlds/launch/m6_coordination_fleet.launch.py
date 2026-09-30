"""
Launch M6 Multi-Agent Path Coordination Fleet across multiple AMRs.

Spawns:
1. Gazebo Harmonic warehouse simulation with N AMRs (default: 5)
2. M3 Task Manager Node
3. M4 Decentralized CBBA Nodes (amr_0 .. amr_{N-1})
4. M6 Decentralized Coordinated Rolling-Horizon Nodes (amr_0 .. amr_{N-1})
   with SpaceTime reservations, PIBT arbitration, and deterministic deadlock recovery.
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
    """Dynamically configure and instantiate CBBA and M6 coordinated RH nodes."""
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

    horizon_steps_str = context.perform_substitution(
        LaunchConfiguration('horizon_steps')
    )
    try:
        horizon_steps = int(horizon_steps_str)
    except ValueError:
        horizon_steps = 10

    execution_window_str = context.perform_substitution(
        LaunchConfiguration('execution_window')
    )
    try:
        execution_window = int(execution_window_str)
    except ValueError:
        execution_window = 4

    replan_rate_str = context.perform_substitution(
        LaunchConfiguration('replan_rate')
    )
    try:
        replan_rate = float(replan_rate_str)
    except ValueError:
        replan_rate = 2.0

    sequencing_heuristic = context.perform_substitution(
        LaunchConfiguration('sequencing_heuristic')
    )
    goal_tolerance_m_str = context.perform_substitution(
        LaunchConfiguration('goal_tolerance_m')
    )
    try:
        goal_tolerance_m = float(goal_tolerance_m_str)
    except ValueError:
        goal_tolerance_m = 0.5

    enable_motion_execution = context.perform_substitution(
        LaunchConfiguration('enable_motion_execution')
    ).lower() in ('true', '1')

    enable_coordination = context.perform_substitution(
        LaunchConfiguration('enable_coordination')
    ).lower() in ('true', '1')

    persistence_threshold_sec_str = context.perform_substitution(
        LaunchConfiguration('persistence_threshold_sec')
    )
    try:
        persistence_threshold_sec = float(persistence_threshold_sec_str)
    except ValueError:
        persistence_threshold_sec = 1.5

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
    if not workload_file:
        candidate_paths = [
            os.path.join(os.getcwd(), 'config', 'workloads', 'workload_medium_priority.yaml'),
            
        ]
        for candidate in candidate_paths:
            if os.path.isfile(candidate):
                workload_file = candidate
                break

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
            msg=(
                f'[M6 Fleet Bringup] Starting {robot_count} CBBA nodes '
                f'(bundle cap: {max_bundle_size})'
            )
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

    # 4. Decentralized M6 Coordinated Rolling Horizon Planner Node per AMR
    actions.append(
        LogInfo(
            msg=(
                f'[M6 Fleet Bringup] Starting {robot_count} Coordinated RH nodes '
                f'(h={horizon_steps}, w={execution_window}, '
                f'coordination={enable_coordination}, '
                f'deadlock_thresh={persistence_threshold_sec}s)'
            )
        )
    )
    for i in range(robot_count):
        robot_id = f'amr_{i}'
        col = i // 5
        row = i % 5
        x_coord = 2.0 if col == 0 else 8.0
        y_coord = 2.0 + row * 3.0

        rh_node = Node(
            package='amr_fleet_core',
            executable='rh_node',
            name='rh_node',
            namespace=robot_id,
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'spawn_x': float(x_coord),
                'spawn_y': float(y_coord),
                'horizon_steps': horizon_steps,
                'execution_window': execution_window,
                'replan_rate': replan_rate,
                'sequencing_heuristic': sequencing_heuristic,
                'goal_tolerance_m': goal_tolerance_m,
                'enable_motion_execution': enable_motion_execution,
                'enable_coordination': enable_coordination,
                'persistence_threshold_sec': persistence_threshold_sec,
                'min_stall_cycles': 3,
            }],
        )
        actions.append(rh_node)

    return actions


def generate_launch_description():
    """Generate launch description with arguments and opaque function."""
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_count',
            default_value='5',
            description='Number of AMRs in fleet (default: 5)',
        ),
        DeclareLaunchArgument(
            'max_bundle_size',
            default_value='4',
            description='Maximum tasks per robot bundle (default: 4)',
        ),
        DeclareLaunchArgument(
            'horizon_steps',
            default_value='10',
            description='Planning horizon h in discrete grid steps (default: 10)',
        ),
        DeclareLaunchArgument(
            'execution_window',
            default_value='4',
            description='Execution window w in discrete grid steps (default: 4)',
        ),
        DeclareLaunchArgument(
            'replan_rate',
            default_value='2.0',
            description='Periodic replan frequency in Hz (default: 2.0)',
        ),
        DeclareLaunchArgument(
            'sequencing_heuristic',
            default_value='PRIORITY_FIRST',
            description=(
                'Task sequencing heuristic (PRIORITY_FIRST, '
                'SHORTEST_PATH_FIRST, DEADLINE_FIRST, BUNDLE_ORDER)'
            ),
        ),
        DeclareLaunchArgument(
            'goal_tolerance_m',
            default_value='0.5',
            description='Arrival tolerance for sub-goals in meters (default: 0.5)',
        ),
        DeclareLaunchArgument(
            'enable_motion_execution',
            default_value='true',
            description='Whether to publish cmd_vel tracking waypoints (default: true)',
        ),
        DeclareLaunchArgument(
            'enable_coordination',
            default_value='true',
            description=(
                'Enable M6 multi-agent path coordination & reservations (default: true)'
            ),
        ),
        DeclareLaunchArgument(
            'persistence_threshold_sec',
            default_value='1.5',
            description=(
                'Persistence threshold in seconds before confirming deadlock (default: 1.5)'
            ),
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


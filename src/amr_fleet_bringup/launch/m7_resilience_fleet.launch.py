"""
Launch M7 Communication Degradation & Fleet Resilience Experiment Fleet across multiple AMRs.

Spawns:
1. Gazebo Harmonic warehouse simulation with N AMRs (default: 5)
2. M3 Task Manager Node
3. M4 Decentralized CBBA Nodes (amr_0 .. amr_{N-1}) with communication impairment & stale tracking
4. M6 Decentralized Coordinated Rolling-Horizon Nodes (amr_0 .. amr_{N-1})
   with communication impairment, stale state tracking, and deterministic deadlock recovery.
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
    """Dynamically configure and instantiate CBBA and M7 resilient RH nodes."""
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

    # M7 Communication Degradation Parameters
    enable_comm_degradation = context.perform_substitution(
        LaunchConfiguration('enable_comm_degradation')
    ).lower() in ('true', '1')

    comm_profile = context.perform_substitution(LaunchConfiguration('comm_profile'))

    comm_latency_ms = float(
        context.perform_substitution(LaunchConfiguration('comm_latency_ms'))
    )
    comm_jitter_ms = float(
        context.perform_substitution(LaunchConfiguration('comm_jitter_ms'))
    )
    comm_loss_prob = float(
        context.perform_substitution(LaunchConfiguration('comm_loss_probability'))
    )
    comm_burst_prob = float(
        context.perform_substitution(LaunchConfiguration('comm_burst_loss_probability'))
    )
    comm_outage_start = float(
        context.perform_substitution(LaunchConfiguration('comm_outage_start_s'))
    )
    comm_outage_dur = float(
        context.perform_substitution(LaunchConfiguration('comm_outage_duration_s'))
    )
    comm_seed = int(
        context.perform_substitution(LaunchConfiguration('comm_seed'))
    )
    stale_thresh = float(
        context.perform_substitution(LaunchConfiguration('stale_threshold_sec'))
    )
    expiry_thresh = float(
        context.perform_substitution(LaunchConfiguration('expiry_threshold_sec'))
    )

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
                f'[M7 Fleet Bringup] Starting {robot_count} CBBA nodes '
                f'(bundle cap: {max_bundle_size}, profile: {comm_profile}, '
                f'deg_enabled: {enable_comm_degradation})'
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
                'enable_comm_degradation': enable_comm_degradation,
                'comm_profile': comm_profile,
                'comm_latency_ms': comm_latency_ms,
                'comm_jitter_ms': comm_jitter_ms,
                'comm_loss_probability': comm_loss_prob,
                'comm_burst_loss_probability': comm_burst_prob,
                'comm_outage_start_s': comm_outage_start,
                'comm_outage_duration_s': comm_outage_dur,
                'comm_seed': comm_seed,
                'stale_threshold_sec': stale_thresh,
                'expiry_threshold_sec': expiry_thresh,
            }],
        )
        actions.append(cbba_node)

    # 4. Decentralized M7 Resilient Coordinated Rolling Horizon Planner Node per AMR
    actions.append(
        LogInfo(
            msg=(
                f'[M7 Fleet Bringup] Starting {robot_count} Resilient RH nodes '
                f'(h={horizon_steps}, w={execution_window}, '
                f'coordination={enable_coordination}, '
                f'profile={comm_profile})'
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
                'grid_resolution': 0.5,
                'goal_tolerance_m': goal_tolerance_m,
                'enable_motion_execution': enable_motion_execution,
                'enable_coordination': enable_coordination,
                'persistence_threshold_sec': persistence_threshold_sec,
                'min_stall_cycles': 3,
                'enable_comm_degradation': enable_comm_degradation,
                'comm_profile': comm_profile,
                'comm_latency_ms': comm_latency_ms,
                'comm_jitter_ms': comm_jitter_ms,
                'comm_loss_probability': comm_loss_prob,
                'comm_burst_loss_probability': comm_burst_prob,
                'comm_outage_start_s': comm_outage_start,
                'comm_outage_duration_s': comm_outage_dur,
                'comm_seed': comm_seed,
                'stale_threshold_sec': stale_thresh,
                'expiry_threshold_sec': expiry_thresh,
            }],
        )
        actions.append(rh_node)

    return actions


def generate_launch_description():
    """Declare launch arguments and return LaunchDescription."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_count',
            default_value='5',
            description='Number of AMRs to deploy in the fleet',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode without GUI',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open RViz2 with multi-agent fleet configuration',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_small',
            description='Gazebo world file name',
        ),
        DeclareLaunchArgument(
            'max_bundle_size',
            default_value='4',
            description='Maximum number of tasks an AMR can bid for',
        ),
        DeclareLaunchArgument(
            'horizon_steps',
            default_value='10',
            description='Planning horizon h in discrete steps',
        ),
        DeclareLaunchArgument(
            'execution_window',
            default_value='4',
            description='Execution window w in discrete steps',
        ),
        DeclareLaunchArgument(
            'replan_rate',
            default_value='2.0',
            description='Rate (Hz) at which rolling-horizon replans occur',
        ),
        DeclareLaunchArgument(
            'sequencing_heuristic',
            default_value='PRIORITY_FIRST',
            description='Task sequencing heuristic',
        ),
        DeclareLaunchArgument(
            'goal_tolerance_m',
            default_value='0.5',
            description='Distance in meters to consider sub-goal reached',
        ),
        DeclareLaunchArgument(
            'enable_motion_execution',
            default_value='true',
            description='Publish cmd_vel velocities to drive robots',
        ),
        DeclareLaunchArgument(
            'enable_coordination',
            default_value='true',
            description='Enable multi-agent space-time reservations and PIBT arbitration',
        ),
        DeclareLaunchArgument(
            'persistence_threshold_sec',
            default_value='1.5',
            description='Duration in seconds before a wait cycle is classified as deadlock',
        ),
        DeclareLaunchArgument(
            'enable_comm_degradation',
            default_value='true',
            description='Enable M7 communication degradation layer',
        ),
        DeclareLaunchArgument(
            'comm_profile',
            default_value='NORMAL',
            description=(
                'M7 preset profile: NORMAL, LOW_LATENCY, HIGH_LATENCY, JITTER, '
                'LOSS_LOW, LOSS_HIGH, BURST_LOSS, OUTAGE, OUTAGE_RECOVERY, PARTITION'
            ),
        ),
        DeclareLaunchArgument(
            'comm_latency_ms',
            default_value='0.0',
            description='Communication latency in milliseconds',
        ),
        DeclareLaunchArgument(
            'comm_jitter_ms',
            default_value='0.0',
            description='Communication latency jitter spread in milliseconds',
        ),
        DeclareLaunchArgument(
            'comm_loss_probability',
            default_value='0.0',
            description='Probability of independent message loss [0.0 - 1.0]',
        ),
        DeclareLaunchArgument(
            'comm_burst_loss_probability',
            default_value='0.0',
            description='Probability of burst loss events [0.0 - 1.0]',
        ),
        DeclareLaunchArgument(
            'comm_outage_start_s',
            default_value='0.0',
            description='Time in seconds when communication outage begins',
        ),
        DeclareLaunchArgument(
            'comm_outage_duration_s',
            default_value='0.0',
            description='Duration in seconds of communication outage',
        ),
        DeclareLaunchArgument(
            'comm_seed',
            default_value='42',
            description='Random seed for deterministic degradation sequences',
        ),
        DeclareLaunchArgument(
            'stale_threshold_sec',
            default_value='1.5',
            description='Age in seconds when information is classified as STALE',
        ),
        DeclareLaunchArgument(
            'expiry_threshold_sec',
            default_value='4.0',
            description='Age in seconds when remote reservations are EXPIRED',
        ),
        DeclareLaunchArgument(
            'workload_file',
            default_value='',
            description='Path to workload YAML file',
        ),
        DeclareLaunchArgument(
            'launch_simulation',
            default_value='true',
            description='Launch Gazebo Harmonic simulation and spawn AMRs',
        ),
        OpaqueFunction(function=launch_setup),
    ])


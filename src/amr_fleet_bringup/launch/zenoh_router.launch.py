"""Launch description for rmw_zenoh router daemon."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    log_info = LogInfo(msg='Launching Zenoh router daemon (rmw_zenohd)...')
    zenoh_process = ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        output='screen',
    )

    return LaunchDescription([
        log_info,
        zenoh_process,
    ])


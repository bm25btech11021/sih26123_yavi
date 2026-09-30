"""Launch description for spawning a single parameterized AMR into Gazebo Harmonic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='amr_0',
        description='Unique robot name / namespace',
    )
    x_arg = DeclareLaunchArgument('x', default_value='2.0', description='Initial X position')
    y_arg = DeclareLaunchArgument('y', default_value='2.0', description='Initial Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.15', description='Initial Z position')
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial Yaw orientation',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse_m9_v2',
        description='Gazebo simulation world name',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process URDF/Xacro
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
            'prefix:=', robot_name, '/',
        ]
    )

    # Robot State Publisher in robot namespace
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spawn entity via ros_gz_sim create
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name=[robot_name, '_spawner'],
        output='screen',
        arguments=[
            '-world', world,
            '-name', robot_name,
            '-topic', [robot_name, '/robot_description'],
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
            '-allow_renaming', 'false',
        ],
    )

    # Bridge topics between Gazebo and ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=[robot_name, '_bridge'],
        output='screen',
        arguments=[
            # cmd_vel (ROS -> GZ)
            ['/', robot_name, '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
            # odometry (GZ -> ROS)
            ['/', robot_name, '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
            # 2D lidar scan (GZ -> ROS)
            ['/', robot_name, '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            # TF (GZ -> ROS)
            ['/', robot_name, '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            # Joint states (GZ -> ROS)
            ['/', robot_name, '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        ],
        remappings=[
            (['/', robot_name, '/tf'], '/tf'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        spawn_entity_node,
        bridge_node,
    ])


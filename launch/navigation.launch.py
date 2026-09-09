"""Start the wheel base and Nav2 navigation servers.

Start the ZED (or another localization and obstacle-sensor stack) separately.
That stack must provide map -> odom and the PointCloud2 topic configured in
config/nav2_params.yaml before sending Nav2 a goal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('experiment_nav2')
    params_file = LaunchConfiguration('params_file')
    robot_config = LaunchConfiguration('robot_config')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': params_file,
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([package_share, 'config', 'nav2_params.yaml']),
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value=PathJoinSubstitution([package_share, 'config', 'robot_nav2.yaml']),
        ),
        # Keep joystick disabled in the robot_config while Nav2 owns /cmd_vel.
        Node(
            package='ddsm115_controller', executable='velocity_control',
            name='velocity_control_node', output='screen', parameters=[robot_config],
        ),
        Node(
            package='ddsm115_controller', executable='two_wheels_robot',
            name='two_wheels_robot_node', output='screen', parameters=[robot_config],
        ),
        nav2,
    ])

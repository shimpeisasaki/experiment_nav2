"""DDSM115 + ZED Mini sensor inspection, without Nav2 or GNSS."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('experiment_nav2')
    robot_config = LaunchConfiguration('robot_config')
    model = PathJoinSubstitution([share, 'urdf', 'experiment_robot.urdf.xacro'])
    return LaunchDescription([
        DeclareLaunchArgument('use_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_zed', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_joystick', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('serial_number', default_value='0'),
        DeclareLaunchArgument('robot_config', default_value=PathJoinSubstitution([
            share, 'config', 'robot_nav2.yaml'])),
        DeclareLaunchArgument('zed_config', default_value=PathJoinSubstitution([
            share, 'config', 'zed_sensors.yaml'])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', model]), value_type=str)}], output='screen'),
        Node(package='ddsm115_controller', executable='velocity_control',
             name='velocity_control_node', parameters=[robot_config], output='screen',
             condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='ddsm115_controller', executable='two_wheels_robot',
             name='two_wheels_robot_node',
             parameters=[robot_config, {
                 'enable_joystick': LaunchConfiguration('enable_joystick'),
                 'pub_tf': True}],
             output='screen', condition=IfCondition(LaunchConfiguration('use_base'))),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_joystick'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_zed')),
            launch_arguments={
                'camera_model': 'zedm', 'camera_name': 'zed', 'namespace': '',
                'node_name': 'zed_node',
                'serial_number': LaunchConfiguration('serial_number'),
                'publish_urdf': 'false', 'publish_tf': 'false',
                'publish_map_tf': 'false', 'publish_imu_tf': 'true',
                'enable_gnss': 'false', 'enable_ipc': 'false',
                'use_sim_time': 'false',
                'ros_params_override_path': LaunchConfiguration('zed_config'),
            }.items()),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([
                 share, 'rviz', 'experiment_robot.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])

"""Run a fixed cmd_vel odometry test and optionally record its rosbag."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('experiment_nav2')
    robot_config = LaunchConfiguration('robot_config')
    manual_config = LaunchConfiguration('manual_config')
    zed_config = LaunchConfiguration('zed_config')
    command_test = Node(
        package='ddsm115_controller', executable='cmd_vel_test', name='cmd_vel_test',
        parameters=[{
            # Explicit types prevent e.g. duration:=20 from being interpreted
            # as an integer by the launch parameter parser.
            'linear_speed': ParameterValue(LaunchConfiguration('linear_speed'), value_type=float),
            'angular_speed': ParameterValue(LaunchConfiguration('angular_speed'), value_type=float),
            'start_delay': ParameterValue(LaunchConfiguration('start_delay'), value_type=float),
            'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float),
            'stop_duration': ParameterValue(LaunchConfiguration('stop_duration'), value_type=float),
            'command_topic': '/cmd_vel_test',
        }], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value=PathJoinSubstitution([
            share, 'config', 'robot_nav2.yaml'])),
        DeclareLaunchArgument(
            'left_usb_dev',
            default_value='/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BD9133ABCD-if06',
            description='CH4 serial device for left motor ID 2'),
        DeclareLaunchArgument(
            'right_usb_dev',
            default_value='/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BD9133ABCD-if04',
            description='CH3 serial device for right motor ID 1'),
        DeclareLaunchArgument('manual_config', default_value=PathJoinSubstitution([
            share, 'config', 'manual_control.yaml'])),
        DeclareLaunchArgument('use_zed', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('serial_number', default_value='10028118'),
        DeclareLaunchArgument('zed_config', default_value=PathJoinSubstitution([
            share, 'config', 'zed_sensors.yaml'])),
        DeclareLaunchArgument('linear_speed', default_value='0.3'),
        DeclareLaunchArgument('angular_speed', default_value='0.0'),
        DeclareLaunchArgument('start_delay', default_value='3.0'),
        DeclareLaunchArgument('duration', default_value='10.0'),
        DeclareLaunchArgument('stop_duration', default_value='1.0'),
        DeclareLaunchArgument('record_bag', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('bag_name', default_value='odom_calibration'),
        Node(package='ddsm115_controller', executable='velocity_control',
             name='velocity_control_node',
             parameters=[robot_config, {
                 'left_usb_dev': LaunchConfiguration('left_usb_dev'),
                 'right_usb_dev': LaunchConfiguration('right_usb_dev'),
             }],
             output='screen'),
        Node(package='ddsm115_controller', executable='two_wheels_robot',
             name='two_wheels_robot_node',
             parameters=[robot_config, {'enable_joystick': False, 'pub_tf': True}],
             output='screen'),
        command_test,
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[manual_config], output='screen',
             remappings=[('cmd_vel', 'cmd_vel_test'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='calibration_velocity_smoother_lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': ['velocity_smoother']}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'zed_sensors.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_zed')),
            launch_arguments={
                'serial_number': LaunchConfiguration('serial_number'),
                'zed_config': zed_config,
            }.items()),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_name'),
                 '/cmd_vel_test', '/cmd_vel', '/odom', '/zed/zed_node/odom',
                 '/ddsm115/rpm_cmd', '/ddsm115/rpm_fb', '/ddsm115/online_id',
                 '/ddsm115/error', '/ddsm115/cur_fb', '/ddsm115/temp_fb', '/tf', '/tf_static'],
            output='screen', condition=IfCondition(LaunchConfiguration('record_bag'))),
        RegisterEventHandler(
            OnProcessExit(
                target_action=command_test,
                on_exit=[
                    LogInfo(msg='Calibration command completed; stopping rosbag and base.'),
                    EmitEvent(event=Shutdown(reason='Calibration command completed')),
                ])),
    ])

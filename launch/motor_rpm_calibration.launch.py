"""Single lifted-wheel RPM test with automatic rosbag recording and shutdown."""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    share = Path(get_package_share_directory('experiment_nav2'))
    motor = LaunchConfiguration('motor').perform(context)
    direction = LaunchConfiguration('direction').perform(context)
    rpm = LaunchConfiguration('rpm').perform(context)
    name = LaunchConfiguration('bag_name').perform(context)
    destination = Path(name or
        f'rpm_{motor}_{direction}_{rpm}_{datetime.now():%Y%m%d_%H%M%S}').expanduser().absolute()
    if destination.exists():
        raise RuntimeError(f'Bag already exists: {destination}. Choose a new bag_name.')

    recorder = ExecuteProcess(cmd=[
        'ros2', 'bag', 'record', '-o', str(destination), '/ddsm115/rpm_cmd',
        '/ddsm115/rpm_fb', '/ddsm115/online_id', '/ddsm115/error',
        '/ddsm115/cur_fb', '/ddsm115/temp_fb'], output='screen',
        sigterm_timeout='15', sigkill_timeout='5')
    test = Node(package='experiment_nav2', executable='motor_rpm_test', output='screen',
                parameters=[{
                    'motor': motor, 'direction': direction,
                    'rpm': ParameterValue(LaunchConfiguration('rpm'), value_type=int),
                    'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float),
                }])
    stop = lambda event, ctx: [] if ctx.is_shutdown else [
        EmitEvent(event=Shutdown(reason='RPM test process ended'))]
    return [
        RegisterEventHandler(OnProcessExit(target_action=test, on_exit=stop)),
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=stop)),
        recorder,
        Node(package='ddsm115_controller', executable='velocity_control',
             name='velocity_control_node', output='screen',
             parameters=[LaunchConfiguration('robot_config')],
             respawn=True, respawn_delay=2.0),
        test,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('experiment_nav2'))
    return LaunchDescription([
        DeclareLaunchArgument('motor', default_value='right', choices=['left', 'right']),
        DeclareLaunchArgument('direction', default_value='forward', choices=['forward', 'reverse']),
        DeclareLaunchArgument('rpm', default_value='90'),
        DeclareLaunchArgument('duration', default_value='10.0'),
        DeclareLaunchArgument('bag_name', default_value=''),
        DeclareLaunchArgument('robot_config', default_value=str(share / 'config/robot_nav2.yaml')),
        OpaqueFunction(function=setup),
    ])

"""Measured motion presets with wheel/IMU EKF and automatic rosbag lifecycle."""
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    share = Path(get_package_share_directory('experiment_nav2'))
    test = LaunchConfiguration('test').perform(context)
    name = LaunchConfiguration('bag_name').perform(context)
    destination = Path(name or f'odom_{test}_{datetime.now():%Y%m%d_%H%M%S}').expanduser().absolute()
    if destination.exists():
        raise RuntimeError(f'Bag already exists: {destination}. Choose a new bag_name.')
    recorder = ExecuteProcess(cmd=[
        'ros2', 'bag', 'record', '-o', str(destination),
        '/cmd_vel', '/wheel/odom', '/odom', '/zed/zed_node/imu/data', '/imu/ekf',
        '/localization/imu_status', '/ddsm115/rpm_cmd', '/ddsm115/rpm_fb',
        '/ddsm115/online_id', '/ddsm115/error', '/tf', '/tf_static'],
        output='screen', sigterm_timeout='15', sigkill_timeout='5')
    motion = Node(package='experiment_nav2', executable='odom_test',
                  parameters=[{'test': test}], output='screen')
    return [
        RegisterEventHandler(OnProcessExit(target_action=motion,
            on_exit=lambda event, ctx: [] if ctx.is_shutdown else [
                EmitEvent(event=Shutdown(reason='Motion test ended; closing bag and base'))])),
        RegisterEventHandler(OnProcessExit(target_action=recorder,
            on_exit=lambda event, ctx: [] if ctx.is_shutdown else [
                EmitEvent(event=Shutdown(reason='Recorder exited; stopping test'))])),
        recorder,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/bringup.launch.py')),
            launch_arguments={'enable_joystick': 'false', 'use_lidar': 'false',
                              'odom_source': 'wheel',
                              'rviz': 'false', 'use_zed': LaunchConfiguration('use_zed')}.items()),
        motion,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('test', default_value='straight',
                              choices=['straight', 'straight_12m', 'left_arc', 'right_arc', 'spin']),
        DeclareLaunchArgument('bag_name', default_value='',
                              description='Default: unique timestamped bag in current directory'),
        DeclareLaunchArgument('use_zed', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])

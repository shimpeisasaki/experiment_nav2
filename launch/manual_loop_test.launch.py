"""Manual indoor loop with wheel/EKF/VIO recording; no autonomous commands."""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    share = Path(get_package_share_directory('experiment_nav2'))
    name = LaunchConfiguration('bag_name').perform(context)
    destination = Path(name or f'manual_loop_{datetime.now():%Y%m%d_%H%M%S}').expanduser().absolute()
    if destination.exists():
        raise RuntimeError(f'Bag already exists: {destination}')
    duration = float(LaunchConfiguration('duration').perform(context))
    if not 0 <= duration < float('inf'):
        raise RuntimeError('duration must be finite and non-negative')
    # Estimated trajectories plus the scan and TF needed for offline 2D SLAM.
    # IMU remains active for EKF/VIO, but is not needed to replay these estimates.
    topics = ['/wheel/odom', '/wheel_odom', '/odom', '/zed/zed_node/odom',
              '/scan', '/tf', '/tf_static']
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', str(destination)] + topics,
        output='screen', sigterm_timeout='30', sigkill_timeout='5')
    actions = [
        RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=lambda event, ctx: [] if ctx.is_shutdown else [
                EmitEvent(event=Shutdown(reason='Recorder exited; stopping manual test'))])),
        recorder,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/bringup.launch.py')),
            launch_arguments={
                'use_base': 'true', 'use_zed': 'true', 'enable_joystick': 'true',
                'odom_source': 'vio',
                'use_lidar': 'true',
                'rviz': LaunchConfiguration('rviz'),
                'serial_number': LaunchConfiguration('serial_number'),
                'zed_config': str(share / 'config/zed_vio_test.yaml'),
            }.items()),
        LogInfo(msg=f'Recording manual loop to {destination}. Wait for VIO messages before driving. '
                    'Stop with B, then Ctrl+C to close the bag. VIO is a reference, not ground truth.'),
    ]
    if duration > 0:
        actions.append(TimerAction(period=duration, actions=[
            EmitEvent(event=Shutdown(reason='Manual loop duration elapsed'))]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_name', default_value=''),
        DeclareLaunchArgument('duration', default_value='0',
                              description='Seconds after launch; 0 means stop manually with Ctrl+C'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('serial_number', default_value='10028118'),
        OpaqueFunction(function=setup),
    ])

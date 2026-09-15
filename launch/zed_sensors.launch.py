"""Shared ZED Mini configuration for sensor bringup and calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Select the ZED Mini permanently mounted on this robot.  A caller can
        # still override this argument when replacing the camera.
        DeclareLaunchArgument('serial_number', default_value='10028118'),
        DeclareLaunchArgument('zed_config', default_value=PathJoinSubstitution([
            FindPackageShare('experiment_nav2'), 'config', 'zed_sensors.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
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
    ])

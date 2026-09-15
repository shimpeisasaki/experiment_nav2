"""Continuous wheel/gyro EKF, including IMU freshness monitoring."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(package='experiment_nav2', executable='imu_guard', output='screen',
             parameters=[{'max_gyro_magnitude': 3.0}]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[PathJoinSubstitution([
                 FindPackageShare('experiment_nav2'), 'config', 'ekf.yaml'])],
             remappings=[('odometry/filtered', '/odom')]),
    ])

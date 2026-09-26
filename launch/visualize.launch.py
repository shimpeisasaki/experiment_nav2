"""Visualize the measured robot geometry without starting the motors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    description = {
        'robot_description': ParameterValue(
            Command(['xacro ', model]), value_type=str
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                FindPackageShare('experiment_cat'), 'urdf', 'experiment_robot.urdf.xacro'
            ]),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('experiment_cat'), 'rviz', 'experiment_robot.rviz'
            ]),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', parameters=[description],
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', rviz_config],
        ),
    ])

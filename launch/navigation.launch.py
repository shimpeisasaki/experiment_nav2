"""Complete LiDAR-based Nav2 bringup for the experiment robot.

With ``slam:=true`` (the default), slam_toolbox publishes map -> odom while
building a map. With ``slam:=false``, pass a saved map YAML path and Nav2
starts map_server + AMCL instead. In both cases this launch owns the base,
robot description, and RPLIDAR S1; do not start those nodes separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('experiment_nav2')
    nav2_share = FindPackageShare('nav2_bringup')
    slam_share = FindPackageShare('slam_toolbox')
    params_file = LaunchConfiguration('params_file')
    robot_config = LaunchConfiguration('robot_config')
    slam = LaunchConfiguration('slam')
    model = PathJoinSubstitution([share, 'urdf', 'experiment_robot.urdf.xacro'])

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_zed', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Saved map YAML. Required when slam:=false.'),
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([share, 'config', 'slam_toolbox.yaml'])),
        DeclareLaunchArgument(
            'robot_config',
            default_value=PathJoinSubstitution([share, 'config', 'robot_nav2.yaml'])),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', model]), value_type=str)}]),
        Node(
            package='ddsm115_controller', executable='velocity_control',
            name='velocity_control_node', output='screen', parameters=[robot_config]),
        Node(
            package='ddsm115_controller', executable='two_wheels_robot',
            name='two_wheels_robot_node', output='screen',
            parameters=[robot_config, {'pub_tf': False, 'enable_joystick': False}],
            remappings=[('/odom', '/wheel/odom')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'ekf.launch.py']))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'zed_sensors.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_zed'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'rplidar_s1.launch.py'])),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': 'laser',
                'lidar_rviz': 'false',
            }.items()),
        Node(
            package='rviz2', executable='rviz2', name='nav2_rviz', output='screen',
            arguments=['-d', PathJoinSubstitution([
                nav2_share, 'rviz', 'nav2_default_view.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz'))),

        # Mapping mode: slam_toolbox supplies map -> odom, then Nav2 supplies
        # planning, control, recovery behaviors, and LiDAR costmaps.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                slam_share, 'launch', 'online_async_launch.py'])),
            condition=IfCondition(slam),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                nav2_share, 'launch', 'navigation_launch.py'])),
            condition=IfCondition(slam),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': params_file,
                'use_composition': 'False',
            }.items()),

        # Saved-map mode: AMCL is the sole owner of map -> odom.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                nav2_share, 'launch', 'bringup_launch.py'])),
            condition=UnlessCondition(slam),
            launch_arguments={
                'slam': 'False',
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': params_file,
                'use_composition': 'False',
            }.items()),
    ])

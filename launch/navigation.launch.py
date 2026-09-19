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
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('experiment_nav2')
    nav2_share = FindPackageShare('nav2_bringup')
    slam_share = FindPackageShare('slam_toolbox')
    params_file = LaunchConfiguration('params_file')
    robot_config = LaunchConfiguration('robot_config')
    manual_config = LaunchConfiguration('manual_config')
    slam = LaunchConfiguration('slam')
    model = PathJoinSubstitution([share, 'urdf', 'experiment_robot.urdf.xacro'])

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_zed', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('odom_source', default_value='vio', choices=['vio', 'wheel']),
        DeclareLaunchArgument('zed_config', default_value=PathJoinSubstitution([
            share, 'config', PythonExpression(["'zed_vio_test.yaml' if '",
                LaunchConfiguration('odom_source'), "' == 'vio' else 'zed_sensors.yaml'"])])),
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
        DeclareLaunchArgument(
            'manual_config',
            default_value=PathJoinSubstitution([share, 'config', 'manual_control.yaml'])),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', model]), value_type=str)}]),
        Node(
            package='ddsm115_controller', executable='velocity_control',
            name='velocity_control_node', output='screen',
            parameters=[robot_config, {'require_safety_heartbeat': True,
                                       'safety_heartbeat_timeout': 0.3}],
            respawn=True, respawn_delay=2.0),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             parameters=[{'autorepeat_rate': 20.0}]),
        Node(package='experiment_nav2', executable='navigation_safety',
             name='navigation_safety', output='screen'),
        Node(package='ddsm115_controller', executable='curvature_teleop',
             name='curvature_teleop', output='screen',
             parameters=[manual_config, {'safety_managed': True}],
             remappings=[('/cmd_vel_teleop', '/cmd_vel_teleop_raw')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother_manual', output='screen',
             parameters=[{
                 'smoothing_frequency': 30.0,
                 'scale_velocities': True,
                 'feedback': 'OPEN_LOOP',
                 'max_velocity': [1.0, 0.0, 1.0],
                 'min_velocity': [-1.0, 0.0, -1.0],
                 'max_accel': [0.5, 0.0, 1.2],
                 'max_decel': [-0.8, 0.0, -2.0],
                 'odom_topic': '/odom',
                 'odom_duration': 0.1,
                 'deadband_velocity': [0.0, 0.0, 0.0],
                 'velocity_timeout': 0.5,
             }],
             remappings=[('cmd_vel', '/cmd_vel_teleop_raw'),
                         ('cmd_vel_smoothed', '/cmd_vel_teleop')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='manual_control_lifecycle_manager', output='screen',
             parameters=[{'autostart': True,
                          'node_names': ['velocity_smoother_manual']}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=[params_file]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='collision_monitor_lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': ['collision_monitor']}]),
        Node(
            package='ddsm115_controller', executable='two_wheels_robot',
            name='two_wheels_robot_node', output='screen',
            parameters=[robot_config, {'pub_tf': False, 'enable_joystick': False}],
            remappings=[('/odom', '/wheel/odom'), ('/cmd_vel', '/cmd_vel_safe')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'odometry.launch.py'])),
            launch_arguments={'odom_source': LaunchConfiguration('odom_source')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'zed_sensors.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_zed')),
            launch_arguments={'zed_config': LaunchConfiguration('zed_config')}.items()),
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

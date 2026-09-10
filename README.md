# experiment_nav2

Nav2 configuration for the DDSM115 differential-drive base.  It uses the ZED
registered point cloud directly in both costmaps, so a 2D LiDAR is not a
runtime dependency.

## Visualize the measured robot

This does not connect to the motors:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select experiment_nav2 --symlink-install
source install/setup.bash
ros2 launch experiment_nav2 visualize.launch.py
```

RViz displays the measured chassis and wheels, the ZED Mini, the GNSS antenna,
TF axes, wheel odometry, and the ZED registered point cloud. The translucent red
rectangle is the provisional Nav2 safety footprint. The caster swivel reference
is at (-0.345, 0, 0.125) m, with 35 mm trail and a 100 mm diameter,
25 mm wide wheel. In the fixed trailing pose its axle is at (-0.380, 0, 0.050) m.
The fork geometry is not modeled; the conservative footprint remains provisional.
The chassis underside is at 50 mm and its thickness is 140 mm (top: 190 mm).

## Sensor inspection bringup (no GNSS, no EKF)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ddsm115_controller experiment_nav2 --symlink-install
source install/setup.bash
ros2 launch experiment_nav2 bringup.launch.py
```

This starts the motor driver, wheel odometry, robot_state_publisher, ZED Mini,
and RViz. It does not start Nav2 or send velocity commands, but the base accepts
external `/cmd_vel` commands. Stop other base/visualization launches first to
avoid duplicate drivers and TF publishers. Use `use_base:=false` for camera-only
inspection, `use_zed:=false` for base-only inspection, or `rviz:=false` for headless
operation. `robot_config`, `zed_config`, and `serial_number` can be overridden.

For manual driving, append `enable_joystick:=true`. The analog right stick controls curvature drive:
vertical sets forward/reverse speed while horizontal sets the curvature of the path. Normal driving
therefore produces arcs rather than pivot turns. `X` selects normal drive (1.0 m/s), `A` selects high
drive (6 km/h), `B` brakes, and `Y` selects freewheel. The joystick publishes `/cmd_vel_teleop`; the
open-loop Nav2 velocity smoother publishes the final `/cmd_vel`. Tune these parameters in
`config/manual_control.yaml`.

## Wheel-odometry calibration tests

Stop the manual bringup and Nav2 before running a test; this launch is the sole publisher of
`/cmd_vel`. It starts the base, a fixed-command publisher, the same Nav2 velocity smoother used by
manual driving, the ZED Mini VIO, and rosbag recording. Both the pre-smoother `/cmd_vel_test` and final
`/cmd_vel`, wheel `/odom`, and ZED `/zed/zed_node/odom` are recorded, so acceleration and deceleration
are included in the analysis. Motor online/error/current/temperature diagnostics are also recorded.
After a finite-duration test, the publisher holds zero for one second and the launch automatically
stops the rosbag and base. Pass `use_zed:=false` only for a base-only diagnostic.

```bash
# Straight test: 0.3 m/s target for 10 s (measure the actual total travel).
ros2 launch experiment_nav2 odom_calibration.launch.py bag_name:=straight_01

# Left/right arcs: 1 m target radius before velocity smoothing.
ros2 launch experiment_nav2 odom_calibration.launch.py \
  angular_speed:=0.3 bag_name:=arc_left_01
ros2 launch experiment_nav2 odom_calibration.launch.py \
  angular_speed:=-0.3 bag_name:=arc_right_01

# One nominal in-place revolution at 0.5 rad/s
ros2 launch experiment_nav2 odom_calibration.launch.py \
  linear_speed:=0.0 angular_speed:=0.5 duration:=12.57 bag_name:=spin_left_01

# Hand-push inspection: in another terminal, first enable freewheel with the service below.
ros2 launch experiment_nav2 odom_calibration.launch.py \
  linear_speed:=0.0 angular_speed:=0.0 duration:=0 bag_name:=hand_push_01

# In a second terminal, after the launch starts:
ros2 service call /ddsm115/set_freewheel std_srvs/srv/SetBool "{data: true}"
# Push by hand, then restore velocity mode before stopping the launch:
ros2 service call /ddsm115/set_freewheel std_srvs/srv/SetBool "{data: false}"
```

Record the measured distance for the straight test, and the measured radius/yaw for each arc or
spin. The bag includes commands, wheel RPM feedback, wheel odometry, and TF for later comparison.

Wheel geometry is radius 0.050 m / track 0.208 m in the experiment config, the
DDSM115 shared GUI config, and the URDF. Change all three together when calibrating.

The wheel node owns `odom -> base_link`; robot_state_publisher owns the camera
mount and internal lens transforms. ZED supplies its calibrated lens-to-IMU TF
(`publish_imu_tf=true`, IPC disabled), but no odometry/map TF. GNSS is excluded.

Inspect the actual topic names and messages:

```bash
ros2 topic list -t
ros2 topic echo /odom --once
ros2 topic echo /zed/zed_node/odom --once
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 param get /two_wheels_robot_node wheel_base
ros2 param get /two_wheels_robot_node R_wheel
ros2 run tf2_ros tf2_echo base_link zed_camera_link
ros2 run tf2_ros tf2_echo base_link zed_imu_link
```

ZED publishes some streams only with subscribers. Wheel odometry is in `odom`,
while VIO is in independent `zed_odom`/`zed_map` coordinates. There is deliberately
no invented static alignment between those origins. Compare relative motion
or align recorded trajectories before overlaying them. RViz defaults to
`base_link` for geometry/cloud inspection; change Fixed Frame to `odom` to view
wheel motion. ZED odometry can be inspected numerically/recorded at this stage.

EKF is the next stage after checking direction, timestamps, covariance, VIO
tracking quality and wheel scale. Fuse selected wheel velocity and VIO pose
components, accounting for the camera offset and independent initial origin.
VIO already uses the ZED IMU: do not assume the VIO and same IMU orientation are
independent measurements. When introducing EKF, disable wheel TF and give EKF
sole ownership of `odom -> base_link`. `map -> odom` is not needed for this
sensor-inspection bringup.

## Navigation TF ownership

`robot_nav2.yaml` disables joystick control and the base is the only publisher
of `odom -> base_link`.  The localization stack must be the only publisher of
`map -> odom`.  Never let ZED positional tracking publish `odom -> camera_link`
with the same `odom` as the wheel base: use ZED as a pose/odometry input to a
fusion/localization stack, or configure its TF output so there is one TF owner.

The ZED frame must have a calibrated static transform from `base_link`; its
point cloud header frame must be reachable from `base_link` in TF.  The default
cloud topic is `/zed/zed_node/point_cloud/cloud_registered`.

## Start Nav2 servers

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select experiment_nav2 --symlink-install
source install/setup.bash
ros2 launch experiment_nav2 navigation.launch.py
```

To use an adjusted motor configuration, add
`robot_config:=/absolute/path/to/robot.yaml` to that command.

This starts only the base and Nav2 navigation servers.  Before setting a goal,
start a localization source that supplies `map -> odom` and a ZED/LiDAR driver
that supplies obstacle data.  A static occupancy map is still needed by the
global planner; create it with a mapping system or provide one from a GIS/map
pipeline.

## Required calibration before autonomous motion

1. Verify the provisional `footprint` in `config/nav2_params.yaml` against the
   full physical envelope including the caster fork.
2. Measure the ZED mounting pose and publish its static TF from `base_link`.
3. Verify `ros2 run tf2_tools view_frames` has one connected tree:
   `map -> odom -> base_link -> zed_*`.
4. Tune wheel radius/base and odometry covariance from straight-line and turn
   tests. Keep Nav2 velocity limits below the verified safe motor limit.

## GNSS + camera + VLM direction

For outdoor operation, use GNSS (preferably RTK) plus IMU/wheel odometry for
global localization, and ZED depth PointCloud2 for near-field collision costmaps.
Use the VLM above Nav2 to turn semantic instructions (for example, "go to the
red gate") into approved map/GNSS goals.  It must not directly command motors:
Nav2, its costmaps, velocity limits, and an independent emergency stop remain
the safety layer.

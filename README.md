# experiment_nav2

[日本語の操作ガイド](README.ja.md)

Nav2 configuration for the DDSM115 differential-drive base. The RPLIDAR S1 is
the primary 2D obstacle and SLAM sensor: its `/scan` feeds both Nav2 costmaps
and slam_toolbox. ZED remains available for VIO and future 3D perception.

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

## Sensor inspection bringup (wheel + IMU EKF, no GNSS)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ddsm115_controller experiment_nav2 --symlink-install
source install/setup.bash
ros2 launch experiment_nav2 bringup.launch.py
```

This starts the motor driver, wheel odometry, robot_state_publisher, ZED Mini,
RPLIDAR S1, and RViz. It does not start Nav2 or send velocity commands, but the base accepts
external `/cmd_vel` commands. Stop other base/visualization launches first to
avoid duplicate drivers and TF publishers. Use `use_base:=false` for camera-only
inspection, `use_zed:=false` for base-only inspection, or `rviz:=false` for headless
operation. Add `use_lidar:=false` to omit the LiDAR. `robot_config`, `zed_config`,
and `serial_number` can be overridden.

For manual driving, append `enable_joystick:=true`. The analog right stick controls curvature drive:
vertical sets forward/reverse speed while horizontal sets the curvature of the path. Normal driving
therefore produces arcs rather than pivot turns. `X` selects normal drive (1.0 m/s), `A` selects high
drive (6 km/h), `B` brakes, and `Y` selects freewheel. The joystick publishes `/cmd_vel_teleop`; the
open-loop Nav2 velocity smoother publishes the final `/cmd_vel`. Tune these parameters in
`config/manual_control.yaml`.

## RPLIDAR S1 standalone test

All launches using `rplidar_s1.launch.py` publish raw data on `/scan_raw` and
filtered data on `/scan`. Vehicle-relative front +/-100 degrees rejects ranges
below 0.1 m; the remaining rear sector rejects ranges below 0.4 m. Rejected
returns become NaN (unknown), not infinity (free-space clearing). Scan metadata
is preserved. Distances are measured from the laser origin. Configure thresholds
in `config/scan_filter.yaml`; its mounting yaw must match the URDF (currently pi).
This also hides real obstacles within the masked distances. Loop bags record
only filtered `/scan`; old recordings are unchanged.

The robot's RPLIDAR S1 is a CP2102 USB serial device (serial `0001`) and runs
at 256000 baud. Install the repository-managed udev rule once so the LiDAR is
always available as `/dev/rplidar`, then launch the standalone scan viewer:

```bash
sudo install -m 644 \
  ~/ros2_ws/src/experiment_nav2/udev/99-experiment-rplidar.rules \
  /etc/udev/rules.d/99-experiment-rplidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

cd ~/ros2_ws
colcon build --packages-select sllidar_ros2 experiment_nav2 --symlink-install
source install/setup.bash
ros2 launch experiment_nav2 rplidar_s1.launch.py
```

This starts only the LiDAR driver and RViz. It publishes `sensor_msgs/LaserScan`
on `/scan` with frame `laser`; it does not start the base or send motor commands.

## Wheel-odometry calibration tests

### Lifted-wheel RPM verification

Secure the raised robot, mark one point on each tire, and stop every other base
launch. Each run waits for both motor feedback and rosbag, counts down ten
seconds, commands one wheel at 90 RPM for 10 seconds, sends zero RPM for three
seconds, and exits automatically. The expected physical count is 15 revolutions.
Run all four directions separately:

```bash
ros2 launch experiment_nav2 motor_rpm_calibration.launch.py motor:=right direction:=forward
ros2 launch experiment_nav2 motor_rpm_calibration.launch.py motor:=right direction:=reverse
ros2 launch experiment_nav2 motor_rpm_calibration.launch.py motor:=left  direction:=forward
ros2 launch experiment_nav2 motor_rpm_calibration.launch.py motor:=left  direction:=reverse
```

Count complete revolutions plus the final fraction of a revolution between the
`Start counting now` log and motor stop. A phone video aimed at the tire mark is
more reliable than counting live. Timestamped bags named `rpm_*` record command,
feedback, online IDs, motor error, current and temperature. Override with, for
example, `rpm:=30 duration:=60.0`; RPM is limited to 1–100 and duration to
1–120 seconds. Array order in the bag is `[right ID1, left ID2]`, while vehicle
forward commands have opposite motor signs because of mounting orientation.

### Wheel + IMU calibration presets (recommended)

Stop all other base/navigation launches. Each command below runs **one** motion,
starts a timestamped rosbag in the current directory, then stops all processes
after deceleration and three seconds of zero commands. Recording and wheel odom
must be discovered before the ten-second pre-motion countdown. No joystick is
started. Motors must be in drive mode, with wheels on the ground.

```bash
ros2 launch experiment_nav2 odom_tests.launch.py test:=straight
ros2 launch experiment_nav2 odom_tests.launch.py test:=straight_12m
ros2 launch experiment_nav2 odom_tests.launch.py test:=left_arc
ros2 launch experiment_nav2 odom_tests.launch.py test:=right_arc
ros2 launch experiment_nav2 odom_tests.launch.py test:=spin
```

Run separately, measuring and repositioning between tests. `straight` is 3 m
and `straight_12m` is 12 m, both at 0.2 m/s; arcs are radius 1 m, 180 degrees
at 0.2 m/s; spin is counterclockwise 360 degrees at 0.3 rad/s. Left is positive
yaw when viewed from above. Linear
acceleration/deceleration is 0.15 m/s²; spin acceleration is 0.3 rad/s². A
trapezoidal command profile includes ramps in the total requested distance/angle.
There is no additional velocity smoother in this launch. Discrete timing and
motor tracking introduce error: recorded `/cmd_vel` is the actual command reference,
and physical distance/angle must be measured independently. EKF pose does not stop
the motion, so wheel scale errors are not hidden by odometry feedback.

Bag includes `/wheel/odom`, fused `/odom`, calibrated ZED IMU, guarded IMU,
fusion status, motor feedback and TF. VIO remains disabled. `use_zed:=false`
allows deliberate wheel-only testing; `bag_name:=straight_01` sets a custom
output directory (existing directories are rejected before hardware starts).

### Legacy constant-duration tests

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

The four-channel RS485 converter maps the forward-facing left wheel to CH4 / motor ID 2 and
the right wheel to CH3 / motor ID 1. The defaults use stable `/dev/serial/by-id/` paths. Override
both `left_usb_dev` and `right_usb_dev` together only when using a different converter.

The physical tire radius is 0.050 m and track is 0.208 m. `R_wheel` is the
effective rolling radius used for velocity conversion; it is calibrated to
0.05065 m from ground tests and must match in the experiment and DDSM115 GUI
configs. The URDF retains the physical 0.050 m tire radius for geometry.
`rpm_feedback_offset: 0.5` compensates the measured signed feedback bias before
mounting-direction conversion. It affects odometry only; exact zero remains zero
and motor commands are unchanged.

## Manual indoor loop: wheel / EKF / VIO comparison

Stop other bringup, navigation, camera, and joystick launches before starting.
This test launches manual control with its normal velocity smoother, LiDAR,
wheel+IMU EKF, and a separate ZED VIO reference. It does not start Nav2 navigation
or SLAM. Recording starts automatically; there is no automatic driving.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch experiment_nav2 manual_loop_test.launch.py
```

Before moving, wait for `Recording...`, then check actual messages in a second
terminal (source the same setup files there):

```bash
ros2 topic echo /zed/zed_node/odom --once --field pose.pose
ros2 topic echo /localization/imu_status --once --qos-durability transient_local
```

Proceed only when VIO messages arrive and IMU status is `wheel_imu`. Mark the
initial axle-center position and heading on the floor. Select X (normal drive),
use the right stick to make one loop, and return to the marked position and
heading. Keep the speed low indoors. Press B, wait about five seconds stationary,
then Ctrl+C once in the launch terminal; wait for `Recording stopped` and process
exit. The bag is saved as `manual_loop_YYYYMMDD_HHMMSS`. No separate recorder
command is needed. `duration:=600` optionally stops all processes after ten
minutes from launch (including startup time); the default has no time limit.

Only six topics are recorded: `/wheel/odom`, `/odom`, `/zed/zed_node/odom`,
`/scan`, `/tf`, and `/tf_static`. Images, motor telemetry, commands and raw IMU
are not recorded. IMU still runs and feeds the live EKF/VIO. The saved estimates
can be compared, but the EKF cannot be recomputed from raw sensors in this bag.
LiDAR is always enabled for this test. Use `ros2 bag info <bag_directory>` after
stopping to verify all six topics have nonzero message counts. Merely listing
a topic does not prove it was recorded.

### Generate a 2D occupancy grid from the recorded loop

Stop all live robot launches. Source the ROS/workspace setup in each terminal.
Start SLAM first, using recorded time and the existing robot-specific parameters:

```bash
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true \
  slam_params_file:=/home/uedalab/ros2_ws/src/experiment_nav2/config/slam_toolbox.yaml
```

In a second terminal, replace `BAG_DIRECTORY` with the saved bag path:

```bash
ros2 bag play BAG_DIRECTORY --clock --rate 0.5 --topics /scan /tf /tf_static
```

Recorded TF supplies `odom -> base_link` and `base_link -> laser`; do not start
another EKF or robot driver during replay. SLAM creates `map -> odom` and `/map`.
After playback and processing finish, leave SLAM running and save in a third
terminal (choose a new output name to avoid overwriting an existing map):

```bash
ros2 run nav2_map_server map_saver_cli -f indoor_loop_map \
  --ros-args -p use_sim_time:=false -p save_map_timeout:=10.0
```

The map saver uses wall time so its timeout still works after the bag clock
stops. The output is `indoor_loop_map.yaml` and `indoor_loop_map.pgm`.

`zed_vio_test.yaml` enables GEN_1 VIO and PERFORMANCE depth processing, without
dense depth/point-cloud publication. This increases GPU use only for this test;
ordinary bringup retains its low-compute profile. Area memory and loop-closure
odometry resets are disabled to measure accumulated drift. ZED uses `zed_odom`
and `zed_map` and does not publish competing odometry TF; VIO is not fused into
the EKF. VIO and EKF share the ZED IMU, so they are not independent ground truth.
For comparison, transform the VIO camera pose to the axle `base_link` using
recorded TF, then align initial poses. Do not compare camera and axle positions
directly on turns or add a fabricated `odom -> zed_odom` static transform.
If ZED restarts during the lap, treat the reset as a new trajectory segment;
prefer repeating the lap for a continuous comparison. Record measured endpoint
position/heading error separately from estimated loop closure.

## USB reconnect recovery

Primary bringup, navigation, and calibration launches restart the motor process
two seconds after either motor remains offline, reopening the stable
`/dev/serial/by-id/` paths. ZED retries device opening for up to six seconds,
then its five-second respawn supervisor starts a clean component container after
the camera USB device re-enumerates. A wheel-link loss inhibits motion: after recovery, reselect `X` or
`A` for joystick driving, or publish a zero `/cmd_vel` before sending a new
navigation command. This prevents a command that was active before the stop
from restarting the robot unexpectedly.

Check that Linux has recreated the devices before expecting recovery:

```bash
udevadm settle --timeout=5
ls -l /dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BD9133ABCD-if04
ls -l /dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BD9133ABCD-if06
lsusb -d 2b03:
```

Recovery log lines include `exiting to reopen USB serial devices` followed by a
new `Start velocity_control_node`, or a new `ros2 launch zed_wrapper` process.
If Linux has not recreated a device, reseat the cable or hub connection; ROS
cannot reopen a device that the OS does not expose.

In bringup/mapping/navigation, EKF owns `odom -> base_link`; robot_state_publisher owns the camera
mount and internal lens transforms. ZED supplies its calibrated lens-to-IMU TF
(`publish_imu_tf=true`, IPC disabled), but no odometry/map TF. GNSS is excluded.

Inspect the actual topic names and messages:

```bash
ros2 topic list -t
ros2 topic echo /odom --once
ros2 topic echo /wheel/odom --once
ros2 topic hz /zed/zed_node/imu/data
ros2 param get /two_wheels_robot_node wheel_base
ros2 param get /two_wheels_robot_node R_wheel
ros2 run tf2_ros tf2_echo base_link zed_camera_link
ros2 run tf2_ros tf2_echo base_link zed_imu_link
```

`config/ekf.yaml` runs a 30 Hz planar EKF: wheel forward velocity and yaw rate
from `/wheel/odom`, plus calibrated ZED gyro yaw rate via `/imu/ekf`. The fused
output is `/odom`. Orientation, acceleration, VIO, depth and GNSS are not fused;
the lightweight ZED profile keeps VIO/depth disabled.

`imu_guard` rejects stale/nonfinite samples, samples without a base-link TF,
and gyro magnitudes above 3.0 rad/s.
After 0.5 seconds without valid IMU, it reports `wheel_only`; valid reception
reports `wheel_imu`. Wheel velocity remains an EKF input throughout, so there is
no switch of odometry origin. Recovery requires IMU messages to resume; this
guard does not restart a terminated camera process. Check status with:

```bash
ros2 topic echo /localization/imu_status --qos-durability transient_local
```

Use `use_zed:=false` for intentional wheel-only operation. Calibration launch
remains separate and publishes raw wheel `/odom`, without this EKF. Never run
it alongside bringup/mapping/navigation. Gyro covariance floor is initially
0.0004 (rad/s)^2; tune using stationary and motion measurements.

Synthetic regression (no hardware; use the same isolated ROS_DOMAIN_ID for both):
start `ros2 launch experiment_nav2 ekf.launch.py`, then run
`python3 src/experiment_nav2/test/check_ekf_fallback.py` in another terminal.

## Navigation TF ownership

`robot_nav2.yaml` disables joystick control and EKF is the only publisher
of `odom -> base_link`.  The localization stack must be the only publisher of
`map -> odom`.  Never let ZED positional tracking publish `odom -> camera_link`
with the same `odom` as the wheel base: use ZED as a pose/odometry input to a
fusion/localization stack, or configure its TF output so there is one TF owner.

The ZED frame must have a calibrated static transform from `base_link`; its
point cloud header frame must be reachable from `base_link` in TF.  The default
cloud topic is `/zed/zed_node/point_cloud/cloud_registered`.

## Start complete LiDAR Nav2

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select experiment_nav2 --symlink-install
source install/setup.bash
ros2 launch experiment_nav2 navigation.launch.py
```

The default is mapping mode (`slam:=true`): it starts the base, robot model,
RPLIDAR S1, slam_toolbox, and Nav2. slam_toolbox is the only publisher of
`map -> odom`; EKF is the only publisher of `odom -> base_link`.
Use RViz's **Nav2 Goal** tool to move while mapping. Do not start
`bringup.launch.py`, the joystick bringup, or `rplidar_s1.launch.py` alongside
this launch because it would duplicate the serial driver, base driver, TF
publishers, or `/cmd_vel` source.

Save a completed map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/site_01
```

For normal navigation on a saved map, use AMCL instead of SLAM:

```bash
ros2 launch experiment_nav2 navigation.launch.py \
  slam:=false map:=~/ros2_ws/maps/site_01.yaml
```

In AMCL mode, set the robot's approximate pose in RViz before giving a goal.
To use an adjusted motor configuration in either mode, add
`robot_config:=/absolute/path/to/robot.yaml`.

## Create a map with the controller

For mapping, use the dedicated manual mapping launch rather than starting
Nav2 navigation. It runs the existing curvature-drive joystick and its velocity
smoother, RPLIDAR S1, robot TF, and slam_toolbox. It does **not** start Nav2, so
there is exactly one publisher of `/cmd_vel`.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch experiment_nav2 mapping.launch.py
```

Use the F710 controls already configured for manual drive: the right stick
drives, `X` selects normal speed, `A` high speed, `B` brake, and `Y` freewheel.
Drive slowly around the perimeter and through the interior with scan overlap;
then save the map and restart in saved-map (`slam:=false`) navigation mode.
The ZED is unnecessary for 2D LiDAR mapping; add `use_zed:=true` only when it
is specifically needed for recording or inspection.

## Required calibration before autonomous motion

1. Verify the provisional `footprint` in `config/nav2_params.yaml` against the
   full physical envelope including the caster fork.
2. Verify the RPLIDAR scan origin is `base_link -> laser = (0.008, 0, 0.192)` m,
   with yaw `pi` radians (180 degrees): scanner +X faces the vehicle rear.
   Restart robot_state_publisher after changing the mounting TF. Earlier bags
   retain their recorded TF and are not corrected by changing the URDF.
3. Verify `ros2 run tf2_tools view_frames` has one connected tree:
   `map -> odom -> base_link -> laser`.
4. Tune wheel radius/base and odometry covariance from straight-line and turn
   tests. Keep Nav2 velocity limits below the verified safe motor limit.

## GNSS + camera + VLM direction

For outdoor operation, use GNSS (preferably RTK) plus IMU/wheel odometry for
global localization, and ZED depth PointCloud2 for near-field collision costmaps.
Use the VLM above Nav2 to turn semantic instructions (for example, "go to the
red gate") into approved map/GNSS goals.  It must not directly command motors:
Nav2, its costmaps, velocity limits, and an independent emergency stop remain
the safety layer.
# Launch configuration maintenance

`bringup.launch.py` and `odom_calibration.launch.py` include the shared
`zed_sensors.launch.py`. Camera model, TF ownership and wrapper options should
be edited there so both workflows stay consistent. Camera parameter values
remain in `config/zed_sensors.yaml`; the public `serial_number` and `zed_config`
arguments are forwarded by both launch files.

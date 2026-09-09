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

## Safety and ownership

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

1. Measure chassis footprint/radius and replace `robot_radius` in
   `config/nav2_params.yaml`.
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

# Tablebot ROS 2 Project Summary

## Goal

Build a ROS 2 Humble stack for the skid-steer Tablebot so it can map a room, localize on the saved map, and use Nav2 to circle a table.

Hardware:

- Intel NUC running Ubuntu 22.04 and ROS 2 Humble
- Teensy 4.1 over USB serial
- SLAMTEC/RPLidar A2M8 over USB serial
- Two-motor skid-steer drive with encoders
- No IMU
- Existing manual override logic retained

## Current Stack

```text
Nav2 / teleop
  -> /cmd_vel
  -> tablebot_bringup/teensy_serial_bridge.py
  -> USB serial: VEL,<linear_mps>,<angular_radps>
  -> Teensy motor controller
  -> motors

Teensy
  -> USB serial: ODOM,<left_ticks>,<right_ticks>,<v_mps>,<w_radps>,<theta_rad>,<remote_state>
  -> teensy_serial_bridge.py
  -> /wheel/odom
  -> robot_localization EKF
  -> /odometry/filtered and odom -> base_link
```

TF tree:

```text
map
  -> odom
    -> base_link
      -> laser_frame
```

Transform ownership:

- `map -> odom`: SLAM Toolbox while mapping, AMCL during saved-map navigation
- `odom -> base_link`: `robot_localization`
- `base_link -> laser_frame`: static transform publisher

Do not run SLAM Toolbox and AMCL at the same time.

## Important Files

```text
~/robot_ws/src/tablebot_bringup/
  launch/base.launch.py
  launch/mapping.launch.py
  launch/navigation.launch.py
  launch/static_tf.launch.py
  config/ekf.yaml
  config/nav2_params.yaml
  config/slam_toolbox_mapping.yaml
  config/table_waypoints.yaml
  config/mapping.rviz
  maps/table_area.yaml
  maps/table_area.pgm
  scripts/teensy_option_a_serial.ino
  tablebot_bringup/teensy_serial_bridge.py
  tablebot_bringup/circle_table_waypoints.py
```

Build after code, launch, config, or map install changes:

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select tablebot_bringup
source install/setup.bash
```

## Teensy Firmware

Firmware path:

```text
~/robot_ws/src/tablebot_bringup/scripts/teensy_option_a_serial.ino
```

Current control mode:

- ROS 2 `/cmd_vel` is converted directly into left/right wheel angular velocity commands.
- Combined velocity/yaw PID from the old controller is bypassed.
- Autonomous PWM uses the full `0-255` range.
- A turn anti-stiction boost raises PWM temporarily if Nav2 commands a turn but encoders show the robot is not rotating.
- Teensy onboard LED reports idle, active autonomous command, manual override, and command timeout.

Serial command:

```text
VEL,<linear_mps>,<angular_radps>
```

Serial telemetry:

```text
ODOM,<left_ticks>,<right_ticks>,<v_mps>,<w_radps>,<theta_rad>,<remote_state>
```

Flash:

```bash
rm -rf /tmp/teensy_option_a_serial_flash
mkdir -p /tmp/teensy_option_a_serial_flash
cp ~/robot_ws/src/tablebot_bringup/scripts/teensy_option_a_serial.ino \
  /tmp/teensy_option_a_serial_flash/teensy_option_a_serial_flash.ino

arduino-cli compile \
  --fqbn teensy:avr:teensy41 \
  /tmp/teensy_option_a_serial_flash \
  --output-dir /tmp/teensy_option_a_serial_flash/build

arduino-cli board list
arduino-cli upload \
  -p <teensy_port_from_board_list> \
  --fqbn teensy:avr:teensy41 \
  /tmp/teensy_option_a_serial_flash
```

Verify telemetry:

```bash
stty -F /dev/ttyACM0 115200 raw -echo
timeout 3 cat /dev/ttyACM0
```

Expected idle stream:

```text
ODOM,0,0,0.000000,0.000000,0.000000,0
```

## Lidar

The Tablebot uses `sllidar_ros2` A2M8 launch files. The Tablebot mapping and navigation launch files default to:

```text
lidar_port:=auto
```

At launch time they pick the first available `/dev/ttyUSB*` and print:

```text
Using SLLidar port: /dev/ttyUSB0
```

Manual override:

```bash
ros2 launch tablebot_bringup navigation.launch.py lidar_port:=/dev/ttyUSB1
```

Direct lidar test:

```bash
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=115200 \
  frame_id:=laser_frame \
  inverted:=false \
  angle_compensate:=true
```

Validate:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## Mapping

Start mapping:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch tablebot_bringup mapping.launch.py
```

Run teleop separately:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.25 \
  -p turn:=1.50
```

Save a map:

```bash
mkdir -p ~/robot_ws/src/tablebot_bringup/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot_ws/src/tablebot_bringup/maps/table_area
```

Current saved map:

```text
~/robot_ws/src/tablebot_bringup/maps/table_area.yaml
~/robot_ws/src/tablebot_bringup/maps/table_area.pgm
```

## Navigation

Start localization, Nav2, base, lidar, and RViz:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch tablebot_bringup navigation.launch.py
```

In RViz:

1. Set the initial pose with `2D Pose Estimate`.
2. Confirm the laser scan aligns with the map.
3. Test one `Nav2 Goal`.
4. Only then run the table waypoint loop.

Useful checks:

```bash
ros2 topic echo /map --once
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Table Waypoint Loop

Waypoint config:

```text
~/robot_ws/src/tablebot_bringup/config/table_waypoints.yaml
```

Runner:

```text
~/robot_ws/src/tablebot_bringup/tablebot_bringup/circle_table_waypoints.py
```

Current runner behavior:

- Reads `table_waypoints.yaml` from the installed package.
- Uses Nav2 Simple Commander.
- Calls `navigator.goThroughPoses(...)`.
- Rebuilds fresh stamped poses every loop.
- Appends the first waypoint to the end of each loop so each route is closed.
- Does not call `navigator.lifecycleShutdown()` because Nav2 is launched externally.

Run:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 run tablebot_bringup circle_table_waypoints
```

Current waypoint settings:

```yaml
enabled: true
frame_id: map
loop: true
repeat_count: 6
```

`repeat_count: 0` with `loop: true` means run forever.

To collect new waypoints:

```bash
ros2 topic echo /clicked_point
```

In RViz, use the `Publish Point` tool and click map-frame points around the table. For smoother continuous motion, prefer 8-12 points with headings tangent to the path rather than 4 sharp corners.

## Current Nav2 Tuning Notes

The robot is a skid-steer tank, so turns need assertive angular commands and enough PWM to overcome track friction.

Current tuning includes:

- Higher bridge clamps in `base.launch.py`
- Higher Nav2 linear/angular limits in `nav2_params.yaml`
- Reduced Nav2 controller/BT rates for the NUC
- Full autonomous PWM range in firmware
- Encoder-based anti-stiction turn boost in firmware

If it clips obstacles, first move waypoints farther from the table. If it sticks in turns, tune the Teensy stiction boost before making Nav2 more aggressive.

## Known Gotchas

- USB port names can change after replugging or hardware work. Launch files auto-select `/dev/ttyUSB*` for lidar, but Teensy still defaults to `/dev/ttyACM0`.
- AMCL will not publish `map -> odom` until an initial pose is set.
- If RViz says `frame [map] does not exist`, set the initial pose or check AMCL/map server logs.
- If lidar works standalone but not in bringup, check the selected lidar port printed by the launch.
- No IMU means yaw depends on wheel odometry and lidar localization; keep obstacle clearance generous.

## Mission Checklist

1. Teensy streams `ODOM,...`.
2. `/wheel/odom` and `/odometry/filtered` publish.
3. `odom -> base_link` and `base_link -> laser_frame` exist.
4. `/scan` publishes in `laser_frame`.
5. Saved map loads.
6. AMCL publishes `map -> odom` after initial pose.
7. One RViz Nav2 goal works.
8. `circle_table_waypoints` completes the requested table loops.

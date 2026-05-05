# Tablebot Bringup

ROS 2 Humble bringup package for the skid-steer Tablebot.

This package starts the Teensy motor/odometry bridge, static lidar TF, EKF odometry, SLAM mapping, saved-map localization, Nav2, RViz, and the table waypoint loop.

For deeper context, see [TABLEBOT_ROS2_PROJECT_SUMMARY.md](TABLEBOT_ROS2_PROJECT_SUMMARY.md).

## Workspace

Expected layout:

```text
~/robot_ws/src/
  tablebot_bringup/
  sllidar_ros2/
```

`sllidar_ros2` should be a sibling package, not nested inside `tablebot_bringup`.

Build:

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select tablebot_bringup
source install/setup.bash
```

## Main Launch Files

Base only:

```bash
ros2 launch tablebot_bringup base.launch.py
```

Mapping:

```bash
ros2 launch tablebot_bringup mapping.launch.py
```

Navigation:

```bash
ros2 launch tablebot_bringup navigation.launch.py
```

Both mapping and navigation launch files auto-select the first available `/dev/ttyUSB*` for the lidar:

```text
lidar_port:=auto
```

Override if needed:

```bash
ros2 launch tablebot_bringup navigation.launch.py lidar_port:=/dev/ttyUSB1
```

## Teensy Firmware

Firmware:

```text
scripts/teensy_option_a_serial.ino
```

Current behavior:

- Receives `VEL,<linear_mps>,<angular_radps>` over USB serial.
- Streams `ODOM,<left_ticks>,<right_ticks>,<v_mps>,<w_radps>,<theta_rad>,<remote_state>`.
- Converts `/cmd_vel` feedforward into left/right motor PWM.
- Uses full autonomous PWM range.
- Includes encoder-based turn anti-stiction boost.
- Keeps manual override behavior.

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

Verify:

```bash
stty -F /dev/ttyACM0 115200 raw -echo
timeout 3 cat /dev/ttyACM0
```

Expected idle stream:

```text
ODOM,0,0,0.000000,0.000000,0.000000,0
```

## Mapping

Start mapping:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch tablebot_bringup mapping.launch.py
```

Run teleop in another terminal:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.25 \
  -p turn:=1.50
```

Save map:

```bash
mkdir -p ~/robot_ws/src/tablebot_bringup/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot_ws/src/tablebot_bringup/maps/table_area
```

## Navigation

Start navigation:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch tablebot_bringup navigation.launch.py
```

In RViz:

1. Set initial pose with `2D Pose Estimate`.
2. Confirm `/scan` aligns with the map.
3. Test one `Nav2 Goal`.
4. Run the waypoint loop.

## Table Waypoint Loop

Config:

```text
config/table_waypoints.yaml
```

Runner:

```text
tablebot_bringup/circle_table_waypoints.py
```

Run while Nav2 is active and localized:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 run tablebot_bringup circle_table_waypoints
```

Current runner behavior:

- Loads `table_waypoints.yaml`.
- Calls `navigator.goThroughPoses(...)`.
- Rebuilds fresh stamped poses every loop.
- Appends the first waypoint at the end of each route to close the loop.
- Leaves Nav2 lifecycle management to the launch file.

Collect waypoint coordinates from RViz:

```bash
ros2 topic echo /clicked_point
```

Use the RViz `Publish Point` tool and add the resulting `x`/`y` values to `config/table_waypoints.yaml`.

## Useful Checks

```bash
ros2 topic hz /scan
ros2 topic echo /map --once
ros2 topic echo /wheel/odom --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

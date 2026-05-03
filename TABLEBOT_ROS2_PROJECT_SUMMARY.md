# Tablebot ROS 2 Project Summary

## Goal

Build a ROS 2 Humble stack for the skid-steer tablebot so it can:

1. Drive from standard ROS 2 `/cmd_vel` commands.
2. Publish wheel odometry from the Teensy/encoder base.
3. Build and save a 2D map with RPLidar A2M7 and SLAM Toolbox.
4. Localize against the saved map.
5. Use Nav2 waypoints to circle a table indefinitely.

Hardware:

- Intel NUC running Ubuntu 22.04 and ROS 2 Humble.
- Teensy 4.1 over USB serial.
- RPLidar A2M7 over USB serial.
- Two-motor skid-steer drive with wheel encoders.
- No IMU.
- Existing RC/Pixhawk manual override logic is retained.

## Architecture

The Teensy is a low-level motor and encoder controller. ROS 2 runs on the NUC.

```text
ROS 2 teleop / Nav2
  -> /cmd_vel
  -> teensy_serial_bridge.py
  -> USB serial: VEL,<linear_mps>,<angular_radps>
  -> Teensy 4.1
  -> motor PWM

Teensy 4.1
  -> USB serial: ODOM,<left_ticks>,<right_ticks>,<v_mps>,<w_radps>,<theta_rad>,<remote_state>
  -> teensy_serial_bridge.py
  -> /wheel/odom
  -> robot_localization EKF
  -> odom -> base_link
```

The RPLidar publishes `/scan`. SLAM Toolbox uses `/scan`, wheel odometry, and TF while mapping. AMCL and Nav2 use the saved map for navigation.

## TF Contract

Target tree:

```text
map
  -> odom
    -> base_link
      -> laser_frame
```

Transform ownership:

- `map -> odom`: SLAM Toolbox during mapping, AMCL during saved-map navigation.
- `odom -> base_link`: `robot_localization` EKF.
- `base_link -> laser_frame`: static transform publisher.

Do not run two publishers for the same transform. Do not run SLAM Toolbox and AMCL simultaneously if both publish `map -> odom`.

## Robot Geometry

Coordinate assumptions:

- `base_link` is at the center of the robot footprint on the floor plane.
- Positive `x` points forward.
- Positive `y` points left.
- Positive `z` points upward.
- `laser_frame` has zero roll, pitch, and yaw relative to `base_link`.

Robot dimensions:

```text
Length: 27 in = 0.6858 m
Width:  14.5 in = 0.3683 m
```

Nav2 footprint:

```yaml
footprint: "[[0.3429, 0.1842], [0.3429, -0.1842], [-0.3429, -0.1842], [-0.3429, 0.1842]]"
```

RPLidar A2M7 mounting:

```text
base_link -> laser_frame:
x     = 0.1016
y     = 0.0000
z     = 0.3302
roll  = 0.0000
pitch = 0.0000
yaw   = 0.0000
```

Use this transform in `static_transform_publisher`.

## Teensy Firmware

Current firmware:

```text
teensy_option_a_serial.ino
```

Expected package location once this is moved into the ROS workspace:

```text
~/robot_ws/src/tablebot_bringup/scripts/teensy_option_a_serial.ino
```

Responsibilities:

- Read RC/Pixhawk channels.
- Preserve manual override logic.
- Count left/right quadrature encoder ticks.
- Accept serial velocity commands from the NUC.
- Run onboard skid-steer low-level control.
- Drive motor PWM outputs.
- Stream odometry telemetry to the NUC.

Serial command:

```text
VEL,<linear_mps>,<angular_radps>
```

Serial telemetry:

```text
ODOM,<left_ticks>,<right_ticks>,<v_mps>,<w_radps>,<theta_rad>,<remote_state>
```

Reset command:

```text
RESET_ODOM
```

Remote states:

```text
NO_REMOTE     = 0
REMOTE_AUTO   = 1
REMOTE_MANUAL = 2
```

Behavior:

- `REMOTE_MANUAL`: RC/Pixhawk directly commands left and right motor PWM.
- `REMOTE_AUTO`: NUC/autonomous commands are allowed.
- `NO_REMOTE`: NUC/autonomous commands are allowed, matching the existing robot behavior.

Safety note: changing `NO_REMOTE` to stop would be reasonable later, but it intentionally changes behavior and should be done as a separate decision.

Firmware constants to tune on the real robot:

```cpp
const int LEFT_MOTOR_POLARITY = -1;
const int RIGHT_MOTOR_POLARITY = 1;
const int LEFT_ENCODER_POLARITY = 1;
const int RIGHT_ENCODER_POLARITY = 1;
float bDR = 1.6 / 3.28;      // effective platform width, meters
float rNominalDR = 0.054;    // nominal wheel radius, meters
float eTickDR = 16000;       // encoder ticks per meter
float omega_max = 15.0;      // max wheel angular velocity command
```

Motor polarity is applied at the H-bridge output layer so a positive wheel command means forward robot motion. Current wiring needs the left motor inverted and the right motor non-inverted.

Encoder polarity is separate from motor polarity. If a wheel is physically moving forward but its raw tick topic decreases, flip that wheel's encoder polarity constant.

Control safety behavior:

- Explicit zero commands, `VEL,0.0000,0.0000`, immediately clear PID state and set autonomous PWM commands to zero.
- Command timeout also clears PID state and sets autonomous PWM commands to zero.
- A small wheel-command deadband prevents tiny PID outputs from mapping to the minimum nonzero PWM.

The firmware is self-contained. It no longer depends on missing custom `TimeStep.h` or `Integrator.h` libraries; odometry integration is done directly with elapsed time.

Built-in Teensy LED indicators:

- Slow blink: ready/idle.
- Solid on: autonomous command active.
- Fast blink: manual override.
- Very fast blink: command watchdog timeout.

## Arduino CLI And Teensy Tooling

Installed and verified on this NUC:

```text
arduino-cli: /usr/local/bin/arduino-cli
version:     1.4.1
Teensy core: teensy:avr@1.60.0
Teensy 4.1:  teensy:avr:teensy41
udev rules:  /etc/udev/rules.d/00-teensy.rules
```

The user is in the `dialout` and `plugdev` groups. If a new login session does not have serial access, reboot or log out and back in.

Check the board:

```bash
arduino-cli board list
```

Expected when the Teensy is connected:

```text
Teensy 4.1 teensy:avr:teensy41
```

Arduino CLI expects the sketch folder name to match the main `.ino` file. This repo currently keeps the sketch at the repo root, so use a temporary matching folder for compile/upload:

```bash
rm -rf /tmp/teensy_option_a_serial_flash
mkdir -p /tmp/teensy_option_a_serial_flash
cp teensy_option_a_serial.ino /tmp/teensy_option_a_serial_flash/teensy_option_a_serial_flash.ino

arduino-cli compile \
  --fqbn teensy:avr:teensy41 \
  /tmp/teensy_option_a_serial_flash \
  --output-dir /tmp/teensy_option_a_serial_flash/build
```

Flash:

```bash
arduino-cli upload \
  -p <teensy_port_from_board_list> \
  --fqbn teensy:avr:teensy41 \
  /tmp/teensy_option_a_serial_flash
```

The port can change. In the last verified session, `arduino-cli board list` showed `usb2/2-3`, but always use the current Teensy port shown by the board list command.

After flashing, verify telemetry:

```bash
stty -F /dev/ttyACM0 115200 raw -echo
timeout 3 cat /dev/ttyACM0
```

Expected idle stream:

```text
ODOM,0,0,0.000000,0.000000,0.000000,0
```

## ROS 2 Package

Package name:

```text
tablebot_bringup
```

Workspace:

```text
~/robot_ws
```

Expected package structure:

```text
tablebot_bringup/
  package.xml
  setup.py
  setup.cfg
  tablebot_bringup/
    __init__.py
    teensy_serial_bridge.py
    circle_table_waypoints.py
  launch/
    base.launch.py
    static_tf.launch.py
  config/
    ekf.yaml
    slam_toolbox_mapping.yaml
    nav2_params.yaml
  maps/
    table_area.yaml
    table_area.pgm
  scripts/
    teensy_option_a_serial.ino
```

Build after package changes:

```bash
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

## Serial Bridge

Node:

```text
tablebot_bringup/teensy_serial_bridge.py
```

Responsibilities:

- Subscribe to `/cmd_vel`.
- Clamp velocity commands to safe limits.
- Send `VEL,<linear>,<angular>` to the Teensy.
- Read `ODOM,...` telemetry.
- Publish raw encoder ticks on `/wheel/left_ticks` and `/wheel/right_ticks`.
- Publish `/wheel/odom` as `nav_msgs/Odometry`.
- Publish `/teensy/remote_state` as `std_msgs/Int32`.
- Provide a reset service that sends `RESET_ODOM`.

Defaults:

```yaml
port: /dev/ttyACM0
baudrate: 115200
cmd_vel_topic: /cmd_vel
odom_topic: /wheel/odom
odom_frame_id: odom
base_frame_id: base_link
max_linear_x: 0.20
max_angular_z: 0.60
command_timeout_sec: 0.50
send_rate_hz: 20.0
left_ticks_topic: /wheel/left_ticks
right_ticks_topic: /wheel/right_ticks
```

The bridge does not publish TF. `robot_localization` owns `odom -> base_link`.

## Base Bringup

`base.launch.py` should start:

- `teensy_serial_bridge`
- `static_tf.launch.py`
- `robot_localization` EKF

Run:

```bash
ros2 launch tablebot_bringup base.launch.py
```

Expected topics:

```text
/cmd_vel
/wheel/left_ticks
/wheel/right_ticks
/wheel/odom
/odometry/filtered
/teensy/remote_state
/tf
/tf_static
```

Expected EKF behavior:

- Input: `/wheel/odom`
- Output: `/odometry/filtered`
- TF: `odom -> base_link`
- `two_d_mode: true`
- `world_frame: odom`

## Hardware Tests

Check Teensy serial:

```bash
ls /dev/ttyACM*
```

Check RPLidar serial:

```bash
ls /dev/ttyUSB*
```

Run the bridge directly from this repo during early testing:

```bash
source /opt/ros/humble/setup.bash
python3 teensy_serial_bridge.py --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baudrate:=115200 \
  -p max_linear_x:=0.20 \
  -p max_angular_z:=0.60
```

Check odometry:

```bash
ros2 topic echo /wheel/odom
```

Check raw encoder tick telemetry:

```bash
ros2 topic echo /wheel/left_ticks
ros2 topic echo /wheel/right_ticks
```

Check static TF:

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

Test motion with the robot safely on blocks first:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}" --rate 10
```

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## RPLidar

Expected topic:

```text
/scan
```

Expected frame:

```text
laser_frame
```

Launch:

```bash
ros2 launch rplidar_ros view_rplidar_a2m7_launch.py
```

Validate:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

If `/scan.header.frame_id` is not `laser_frame`, either adjust the RPLidar launch parameters or update the static TF child frame to match.

## Mapping Workflow

Terminal 1: base

```bash
ros2 launch tablebot_bringup base.launch.py
```

Terminal 2: lidar

```bash
ros2 launch rplidar_ros view_rplidar_a2m7_launch.py
```

Terminal 3: SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=$HOME/robot_ws/src/tablebot_bringup/config/slam_toolbox_mapping.yaml
```

Terminal 4: RViz

```bash
rviz2
```

RViz:

- Fixed frame: `map`
- Displays: TF, LaserScan `/scan`, Map `/map`, Odometry `/wheel/odom`

Terminal 5: teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive slowly around the table and nearby features.

Save map:

```bash
mkdir -p ~/robot_ws/src/tablebot_bringup/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot_ws/src/tablebot_bringup/maps/table_area
```

Expected output:

```text
table_area.yaml
table_area.pgm
```

## Navigation Workflow

Terminal 1: base

```bash
ros2 launch tablebot_bringup base.launch.py
```

Terminal 2: lidar

```bash
ros2 launch rplidar_ros view_rplidar_a2m7_launch.py
```

Terminal 3: Nav2

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=false \
  map:=$HOME/robot_ws/src/tablebot_bringup/maps/table_area.yaml \
  params_file:=$HOME/robot_ws/src/tablebot_bringup/config/nav2_params.yaml
```

Terminal 4: RViz

```bash
rviz2
```

In RViz:

1. Set fixed frame to `map`.
2. Add `Map`, `TF`, `LaserScan`, `Odometry`, and `/amcl_pose`.
3. Use `2D Pose Estimate` to set the robot's initial pose.
4. Use `Nav2 Goal` to test one autonomous goal.

Do not run the waypoint loop until one clicked Nav2 goal works reliably.

## Waypoint Loop

Node:

```text
tablebot_bringup/circle_table_waypoints.py
```

Run:

```bash
ros2 run tablebot_bringup circle_table_waypoints
```

Waypoint strategy:

- Use 4 to 8 waypoints around the table.
- Keep generous clearance from the table.
- Test one Nav2 goal first.
- Test two waypoints.
- Test a full loop once.
- Then enable indefinite looping.

## Known Risks

- Direction conventions: verify forward motion, positive yaw, and odometry sign before mapping.
- Duplicate TF publishers: only one node should publish each transform.
- No IMU: yaw during skid-steer turns depends on wheel odometry and lidar localization, so keep speeds conservative.
- Waypoints too close to the table: the robot footprint is 0.6858 m by 0.3683 m and skid-steer turns need margin.
- USB device names can change: add stable udev symlinks later for `/dev/tablebot_teensy` and `/dev/tablebot_lidar`.

## Mission Checklist

1. ROS 2 Humble installed.
2. `tablebot_bringup` builds.
3. Teensy firmware compiles and flashes.
4. Teensy streams `ODOM,...` on `/dev/ttyACM0`.
5. `teensy_serial_bridge` publishes `/wheel/odom`.
6. `robot_localization` publishes `odom -> base_link`.
7. Static TF publishes `base_link -> laser_frame`.
8. `/cmd_vel` moves the robot correctly.
9. RPLidar publishes `/scan`.
10. SLAM Toolbox builds `/map`.
11. Map saver writes `table_area.yaml` and `table_area.pgm`.
12. Nav2 launches with the saved map.
13. AMCL publishes `/amcl_pose`.
14. One RViz Nav2 Goal works.
15. Real table-circling waypoints are entered.
16. The waypoint loop circles the table repeatedly.

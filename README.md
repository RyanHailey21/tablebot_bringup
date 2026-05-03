# ROS 2 Teensy Option A Rewrite

Files:

- `teensy_option_a_serial.ino`
  - Flash this to the Teensy 4.1.
  - Removes ROS 1 `rosserial`.
  - Keeps original remote/manual override behavior.
  - Receives `VEL,<linear_mps>,<angular_radps>` from the NUC.
  - Sends `ODOM,<left_ticks>,<right_ticks>,<v_mps>,<w_radps>,<theta_rad>,<remote_state>` to the NUC.
  - Uses the built-in Teensy LED for robot state.

- `teensy_serial_bridge.py`
  - ROS 2 Humble node for the NUC.
  - Subscribes `/cmd_vel`.
  - Publishes `/wheel/odom`.
  - Does not publish TF; use `robot_localization` to publish `odom -> base_link`.

Install Python dependency:

```bash
sudo apt install python3-serial
```

Teensy status indicators:

- Built-in LED slow blink: ready/idle.
- Built-in LED solid on: autonomous command active.
- Built-in LED fast blink: manual override.
- Built-in LED very fast blink: command watchdog timeout.

Recommended ROS 2 package entry point:

```python
entry_points={
    'console_scripts': [
        'teensy_serial_bridge = tablebot_bringup.teensy_serial_bridge:main',
    ],
},
```

Run:

```bash
ros2 run tablebot_bringup teensy_serial_bridge --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baudrate:=115200 \
  -p max_linear_x:=0.20 \
  -p max_angular_z:=0.60
```

Test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.10}, angular: {z: 0.0}}" --rate 10
```

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

#!/usr/bin/env python3

"""
ROS 2 Humble Teensy serial bridge for the Option A firmware.

ROS 2 -> Teensy:
  subscribes geometry_msgs/Twist on /cmd_vel
  sends VEL,<linear_mps>,<angular_radps>\n

Teensy -> ROS 2:
  reads ODOM,<left_ticks>,<right_ticks>,<v_mps>,<w_radps>,<theta_rad>,<remote_state>
  publishes nav_msgs/Odometry on /wheel/odom
  publishes raw encoder counts on /wheel/left_ticks and /wheel/right_ticks

This node does NOT publish TF by default. Let robot_localization publish odom -> base_link.
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_srvs.srv import Empty

try:
    import serial
except ImportError as exc:
    raise RuntimeError("Install python3-serial: sudo apt install python3-serial") from exc


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class TeensySerialBridge(Node):
    def __init__(self):
        super().__init__("teensy_serial_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("max_linear_x", 0.30)
        self.declare_parameter("max_angular_z", 1.00)
        self.declare_parameter("command_timeout_sec", 0.50)
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("publish_remote_state_topic", "/teensy/remote_state")
        self.declare_parameter("left_ticks_topic", "/wheel/left_ticks")
        self.declare_parameter("right_ticks_topic", "/wheel/right_ticks")

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.remote_state_topic = self.get_parameter("publish_remote_state_topic").value
        self.left_ticks_topic = self.get_parameter("left_ticks_topic").value
        self.right_ticks_topic = self.get_parameter("right_ticks_topic").value

        self._lock = threading.Lock()
        self._latest_v = 0.0
        self._latest_w = 0.0
        self._last_cmd_time = self.get_clock().now()

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_odom_time: Optional[rclpy.time.Time] = None

        self._serial = serial.Serial(self.port, self.baudrate, timeout=0.02)
        time.sleep(2.0)  # allow Teensy reset after opening serial

        self._cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._cmd_vel_callback,
            10,
        )

        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._remote_pub = self.create_publisher(Int32, self.remote_state_topic, 10)
        self._left_ticks_pub = self.create_publisher(Int32, self.left_ticks_topic, 10)
        self._right_ticks_pub = self.create_publisher(Int32, self.right_ticks_topic, 10)

        self._reset_srv = self.create_service(Empty, "~/reset_teensy_odom", self._reset_odom_cb)

        self._send_timer = self.create_timer(1.0 / self.send_rate_hz, self._send_command)
        self._read_timer = self.create_timer(0.005, self._read_serial)

        self.get_logger().info(
            f"Connected to Teensy on {self.port} at {self.baudrate}. "
            f"Subscribing {self.cmd_vel_topic}, publishing {self.odom_topic}, "
            f"{self.left_ticks_topic}, and {self.right_ticks_topic}."
        )

    def _cmd_vel_callback(self, msg: Twist):
        v = self._clamp(msg.linear.x, -self.max_linear_x, self.max_linear_x)
        w = self._clamp(msg.angular.z, -self.max_angular_z, self.max_angular_z)

        if not math.isfinite(v):
            v = 0.0
        if not math.isfinite(w):
            w = 0.0

        with self._lock:
            self._latest_v = v
            self._latest_w = w
            self._last_cmd_time = self.get_clock().now()

    def _send_command(self):
        now = self.get_clock().now()

        with self._lock:
            age = (now - self._last_cmd_time).nanoseconds * 1e-9
            if age > self.command_timeout_sec:
                v = 0.0
                w = 0.0
            else:
                v = self._latest_v
                w = self._latest_w

        line = f"VEL,{v:.4f},{w:.4f}\n"
        try:
            self._serial.write(line.encode("ascii"))
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")

    def _read_serial(self):
        try:
            while self._serial.in_waiting:
                raw = self._serial.readline().decode("ascii", errors="replace").strip()
                if raw:
                    self._handle_line(raw)
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial read failed: {exc}")

    def _handle_line(self, line: str):
        parts = line.split(",")

        if len(parts) < 7 or parts[0] != "ODOM":
            return

        try:
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])
            v = float(parts[3])
            w = float(parts[4])
            theta_from_teensy = float(parts[5])
            remote_state = int(parts[6])
        except ValueError:
            self.get_logger().warn(f"Could not parse Teensy line: {line}")
            return

        now = self.get_clock().now()

        if self._last_odom_time is None:
            dt = 0.0
        else:
            dt = (now - self._last_odom_time).nanoseconds * 1e-9

        self._last_odom_time = now

        if dt > 0.0 and dt < 1.0:
            self._theta += w * dt
            self._x += v * math.cos(self._theta) * dt
            self._y += v * math.sin(self._theta) * dt

        # Alternative:
        # If you trust the Teensy's integrated theta more, uncomment this:
        # self._theta = theta_from_teensy

        self._publish_ticks(left_ticks, right_ticks)
        self._publish_odom(now, v, w)
        self._publish_remote_state(remote_state)

    def _publish_odom(self, stamp, v: float, w: float):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self._theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        # Conservative covariance placeholders. Tune these for robot_localization.
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[35] = 0.10

        self._odom_pub.publish(msg)

    def _publish_remote_state(self, remote_state: int):
        msg = Int32()
        msg.data = remote_state
        self._remote_pub.publish(msg)

    def _publish_ticks(self, left_ticks: int, right_ticks: int):
        left_msg = Int32()
        right_msg = Int32()
        left_msg.data = left_ticks
        right_msg.data = right_ticks
        self._left_ticks_pub.publish(left_msg)
        self._right_ticks_pub.publish(right_msg)

    def _reset_odom_cb(self, request, response):
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_odom_time = None

        try:
            self._serial.write(b"RESET_ODOM\n")
            self.get_logger().info("Sent RESET_ODOM to Teensy.")
        except serial.SerialException as exc:
            self.get_logger().error(f"Failed to send RESET_ODOM: {exc}")

        return response

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def destroy_node(self):
        try:
            self._serial.write(b"VEL,0.0000,0.0000\n")
            self._serial.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeensySerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

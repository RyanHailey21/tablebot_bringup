#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

try:
    import yaml
except ImportError as exc:
    raise RuntimeError("Install PyYAML: sudo apt install python3-yaml") from exc


def yaw_to_quaternion(yaw_rad):
    return 0.0, 0.0, math.sin(yaw_rad * 0.5), math.cos(yaw_rad * 0.5)


def load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Waypoint file must contain a YAML mapping: {path}")
    return data


def make_pose(navigator, frame_id, waypoint):
    pose = PoseStamped()
    pose.header.frame_id = waypoint.get("frame_id", frame_id)
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(waypoint["x"])
    pose.pose.position.y = float(waypoint["y"])
    pose.pose.position.z = float(waypoint.get("z", 0.0))

    if "yaw_deg" in waypoint:
        yaw = math.radians(float(waypoint["yaw_deg"]))
    else:
        yaw = float(waypoint.get("yaw", 0.0))

    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def task_result_name(result):
    if result == TaskResult.SUCCEEDED:
        return "SUCCEEDED"
    if result == TaskResult.CANCELED:
        return "CANCELED"
    if result == TaskResult.FAILED:
        return "FAILED"
    return str(result)


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()
    default_waypoints = str(
        Path(get_package_share_directory("tablebot_bringup"))
        / "config"
        / "table_waypoints.yaml"
    )

    navigator.declare_parameter("waypoints_file", default_waypoints)
    waypoints_file = navigator.get_parameter("waypoints_file").value

    try:
        config = load_config(waypoints_file)
        if not bool(config.get("enabled", False)):
            raise RuntimeError(
                f"{waypoints_file} is not enabled yet. Replace the example poses "
                "with real table waypoints, then set enabled: true."
            )

        frame_id = config.get("frame_id", "map")
        loop = bool(config.get("loop", False))
        repeat_count = int(config.get("repeat_count", 1))
        waypoints = config.get("waypoints", [])

        if not waypoints:
            raise RuntimeError(f"No waypoints found in {waypoints_file}")

        navigator.info(
            f"Loaded {len(waypoints)} waypoint(s) from {waypoints_file}. "
            f"Loop: {loop}."
        )
        navigator.info("Waiting for Nav2 to become active...")
        navigator.waitUntilNav2Active()

        loops_done = 0
        last_feedback_log_time = navigator.get_clock().now()
        while rclpy.ok():
            loops_done += 1
            poses = [
                make_pose(navigator, frame_id, waypoint)
                for waypoint in waypoints
            ]

            navigator.info(f"Starting waypoint loop {loops_done}.")
            navigator.followWaypoints(poses)

            while not navigator.isTaskComplete():
                rclpy.spin_once(navigator, timeout_sec=0.1)
                feedback = navigator.getFeedback()
                now = navigator.get_clock().now()
                if (
                    feedback is not None
                    and (now - last_feedback_log_time).nanoseconds >= 2_000_000_000
                ):
                    last_feedback_log_time = now
                    navigator.info(
                        "Waypoint feedback: "
                        f"heading toward waypoint index {feedback.current_waypoint}.",
                    )

            result = navigator.getResult()
            navigator.info(
                f"Waypoint loop {loops_done} finished: {task_result_name(result)}."
            )

            if result == TaskResult.CANCELED:
                return 1
            if result == TaskResult.FAILED:
                if not loop:
                    return 1
                navigator.info("Waypoint task failed; retrying loop.")
                continue

            if not loop:
                return 0
            if repeat_count > 0 and loops_done >= repeat_count:
                return 0

    except KeyboardInterrupt:
        navigator.info("Canceling waypoint task.")
        navigator.cancelTask()
        return 1
    except Exception as exc:
        navigator.error(str(exc))
        return 1
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

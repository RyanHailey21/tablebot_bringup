#!/usr/bin/env python3

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_deg_to_quaternion(yaw_deg):
    yaw = math.radians(float(yaw_deg))
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def make_pose(navigator, frame_id, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    qz, qw = yaw_deg_to_quaternion(yaw_deg)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


def load_waypoint_config():
    pkg = get_package_share_directory("tablebot_bringup")
    path = os.path.join(pkg, "config", "table_waypoints.yaml")

    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main():
    rclpy.init()

    config = load_waypoint_config()

    if not config.get("enabled", True):
        print("Waypoint config is disabled. Exiting.")
        rclpy.shutdown()
        return

    frame_id = config.get("frame_id", "map")
    loop_enabled = bool(config.get("loop", True))
    repeat_count = int(config.get("repeat_count", 1))
    raw_waypoints = config.get("waypoints", [])

    if len(raw_waypoints) < 2:
        print(f"Need at least 2 waypoints. Got {len(raw_waypoints)}.")
        rclpy.shutdown()
        return

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    print(f"Loaded {len(raw_waypoints)} waypoints.")
    print("Mode: through_poses")
    print(f"Loop enabled: {loop_enabled}")
    print(f"Repeat count: {repeat_count}")

    if loop_enabled and repeat_count == 0:
        total_loops = None
    else:
        total_loops = max(1, repeat_count)

    loop_idx = 0

    while rclpy.ok():
        loop_idx += 1

        if total_loops is not None and loop_idx > total_loops:
            print(f"Completed requested {total_loops} loops. Exiting.")
            break

        loop_suffix = f"/{total_loops}" if total_loops else ""
        print(f"Starting loop {loop_idx}{loop_suffix}")

        poses = [
            make_pose(
                navigator,
                frame_id,
                waypoint["x"],
                waypoint["y"],
                waypoint.get("yaw_deg", 0.0),
            )
            for waypoint in raw_waypoints
        ]

        if loop_enabled:
            first = raw_waypoints[0]
            poses.append(
                make_pose(
                    navigator,
                    frame_id,
                    first["x"],
                    first["y"],
                    first.get("yaw_deg", 0.0),
                )
            )

        print(f"Submitting {len(poses)} poses to Nav2.")
        navigator.goThroughPoses(poses)

        while not navigator.isTaskComplete():
            rclpy.spin_once(navigator, timeout_sec=0.1)

        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            print(f"Loop {loop_idx} succeeded.")
            continue

        if result == TaskResult.CANCELED:
            print(f"Loop {loop_idx} canceled. Exiting.")
            break

        if result == TaskResult.FAILED:
            print(f"Loop {loop_idx} failed. Exiting.")
            break

        print(f"Loop {loop_idx} returned unknown result: {result}. Exiting.")
        break

    # Nav2 is launched externally; leave lifecycle management to the launch file.
    rclpy.shutdown()


if __name__ == "__main__":
    main()

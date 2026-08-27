#!/usr/bin/env python3
"""Send one deterministic Nav2 goal and record controller-level metrics."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Twist,
)
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class TrialRecorder(Node):
    def __init__(self) -> None:
        super().__init__("custom_mppi_validation_trial")
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_amcl: tuple[float, float, float] | None = None
        self.latest_odom = (0.0, 0.0)
        self.latest_raw_cmd = (0.0, 0.0)
        self.latest_smooth_cmd = (0.0, 0.0)
        self.latest_scan = math.nan
        self.latest_plan: np.ndarray | None = None
        self.last_odom_xy: tuple[float, float] | None = None
        self.odom_path_length = 0.0
        self.active = False
        self.start_wall = math.nan
        self.rows: list[dict[str, float]] = []
        self.raw_cmd_samples: list[tuple[float, float, float]] = []
        self.smooth_cmd_samples: list[tuple[float, float, float]] = []
        self.scan_samples: list[float] = []
        self.cross_track_samples: list[float] = []

        self.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self.on_amcl,
            20,
        )
        self.create_subscription(
            Odometry,
            "odom",
            self.on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "cmd_vel_nav", self.on_raw_cmd, 50)
        self.create_subscription(Twist, "cmd_vel", self.on_smooth_cmd, 50)
        self.create_subscription(
            LaserScan,
            "scan",
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(NavPath, "plan", self.on_plan, 10)
        self.create_timer(0.05, self.snapshot)

    def elapsed(self) -> float:
        return 0.0 if not self.active else time.monotonic() - self.start_wall

    def on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.latest_amcl = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def refresh_map_pose_from_tf(self) -> bool:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
            )
        except TransformException:
            return False
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.latest_amcl = (
            translation.x,
            translation.y,
            yaw_from_quaternion(rotation),
        )
        if self.active and self.latest_plan is not None:
            point = np.asarray(self.latest_amcl[:2])
            error = float(
                np.min(np.linalg.norm(self.latest_plan - point, axis=1))
            )
            self.cross_track_samples.append(error)
        return True

    def on_odom(self, msg: Odometry) -> None:
        xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        if self.active and self.last_odom_xy is not None:
            self.odom_path_length += math.hypot(
                xy[0] - self.last_odom_xy[0],
                xy[1] - self.last_odom_xy[1],
            )
        self.last_odom_xy = xy
        self.latest_odom = (
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )

    def on_raw_cmd(self, msg: Twist) -> None:
        self.latest_raw_cmd = (msg.linear.x, msg.angular.z)
        if self.active:
            self.raw_cmd_samples.append(
                (self.elapsed(), msg.linear.x, msg.angular.z)
            )

    def on_smooth_cmd(self, msg: Twist) -> None:
        self.latest_smooth_cmd = (msg.linear.x, msg.angular.z)
        if self.active:
            self.smooth_cmd_samples.append(
                (self.elapsed(), msg.linear.x, msg.angular.z)
            )

    def on_scan(self, msg: LaserScan) -> None:
        finite = [
            value
            for value in msg.ranges
            if math.isfinite(value)
            and value >= msg.range_min
            and value <= msg.range_max
        ]
        self.latest_scan = min(finite) if finite else math.nan
        if self.active and math.isfinite(self.latest_scan):
            self.scan_samples.append(self.latest_scan)

    def on_plan(self, msg: NavPath) -> None:
        if msg.poses:
            self.latest_plan = np.asarray(
                [
                    (pose.pose.position.x, pose.pose.position.y)
                    for pose in msg.poses
                ],
                dtype=float,
            )

    def snapshot(self) -> None:
        if not self.active:
            return
        self.refresh_map_pose_from_tf()
        amcl = self.latest_amcl or (math.nan, math.nan, math.nan)
        self.rows.append(
            {
                "wall_time_s": self.elapsed(),
                "amcl_x_m": amcl[0],
                "amcl_y_m": amcl[1],
                "amcl_yaw_rad": amcl[2],
                "odom_vx_mps": self.latest_odom[0],
                "odom_wz_radps": self.latest_odom[1],
                "raw_cmd_vx_mps": self.latest_raw_cmd[0],
                "raw_cmd_wz_radps": self.latest_raw_cmd[1],
                "smoothed_cmd_vx_mps": self.latest_smooth_cmd[0],
                "smoothed_cmd_wz_radps": self.latest_smooth_cmd[1],
                "minimum_scan_range_m": self.latest_scan,
            }
        )


def total_variation(samples: list[tuple[float, float, float]], index: int) -> float:
    if len(samples) < 2:
        return 0.0
    return float(
        sum(
            abs(samples[i][index] - samples[i - 1][index])
            for i in range(1, len(samples))
        )
    )


def mean_rate(samples: list[tuple[float, float, float]]) -> float:
    if len(samples) < 2:
        return math.nan
    duration = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / duration if duration > 0.0 else math.nan


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", choices=("custom", "official"), required=True)
    parser.add_argument("--trial", default="smoke_01")
    parser.add_argument("--goal-x", type=float, default=1.5)
    parser.add_argument("--goal-y", type=float, default=0.5)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(__file__).resolve().parent / "results/trials",
    )
    args = parser.parse_args()

    output_dir = (args.output_root / args.variant / args.trial).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = TrialRecorder()
    status = GoalStatus.STATUS_UNKNOWN
    message = ""
    goal_handle = None
    try:
        deadline = time.monotonic() + 90.0
        action_ready = False
        transform_ready = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            action_ready = node.action.server_is_ready()
            transform_ready = node.refresh_map_pose_from_tf()
            if action_ready and transform_ready:
                break
        else:
            raise TimeoutError(
                "Nav2 readiness failed: "
                f"action_server={action_ready}, map_to_base_tf={transform_ready}"
            )

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        goal.pose.pose.orientation.z = math.sin(args.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(args.goal_yaw / 2.0)

        future = node.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal was rejected")

        node.active = True
        node.start_wall = time.monotonic()
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        if result_future.done():
            result = result_future.result()
            status = int(result.status)
            message = "action completed"
        else:
            message = f"timeout after {args.timeout:.1f}s"
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
        node.active = False
    except Exception as exc:
        message = f"{type(exc).__name__}: {exc}"
    finally:
        final_pose = node.latest_amcl or (math.nan, math.nan, math.nan)
        duration = (
            time.monotonic() - node.start_wall
            if math.isfinite(node.start_wall)
            else 0.0
        )
        final_goal_error = math.hypot(
            final_pose[0] - args.goal_x,
            final_pose[1] - args.goal_y,
        )
        min_scan = (
            min(node.scan_samples) if node.scan_samples else math.nan
        )
        summary = {
            "variant": args.variant,
            "trial": args.trial,
            "goal": {
                "x": args.goal_x,
                "y": args.goal_y,
                "yaw": args.goal_yaw,
            },
            "status_code": status,
            "succeeded": status == GoalStatus.STATUS_SUCCEEDED,
            "message": message,
            "duration_wall_s": duration,
            "final_pose": {
                "x": final_pose[0],
                "y": final_pose[1],
                "yaw": final_pose[2],
            },
            "final_goal_error_m": final_goal_error,
            "odom_path_length_m": node.odom_path_length,
            "minimum_scan_range_m": min_scan,
            "approx_minimum_body_clearance_m": (
                min_scan - 0.22 if math.isfinite(min_scan) else math.nan
            ),
            "mean_cross_track_error_m": (
                float(np.mean(node.cross_track_samples))
                if node.cross_track_samples
                else math.nan
            ),
            "p95_cross_track_error_m": (
                float(np.percentile(node.cross_track_samples, 95))
                if node.cross_track_samples
                else math.nan
            ),
            "raw_cmd_rate_hz": mean_rate(node.raw_cmd_samples),
            "smoothed_cmd_rate_hz": mean_rate(node.smooth_cmd_samples),
            "raw_vx_total_variation": total_variation(
                node.raw_cmd_samples, 1
            ),
            "raw_wz_total_variation": total_variation(
                node.raw_cmd_samples, 2
            ),
            "smoothed_vx_total_variation": total_variation(
                node.smooth_cmd_samples, 1
            ),
            "smoothed_wz_total_variation": total_variation(
                node.smooth_cmd_samples, 2
            ),
            "timeseries_rows": len(node.rows),
        }
        with (output_dir / "summary.json").open(
            "w", encoding="utf-8"
        ) as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2)
        if node.rows:
            with (output_dir / "timeseries.csv").open(
                "w", newline="", encoding="utf-8"
            ) as stream:
                writer = csv.DictWriter(
                    stream, fieldnames=list(node.rows[0].keys())
                )
                writer.writeheader()
                writer.writerows(node.rows)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

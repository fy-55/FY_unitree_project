#!/usr/bin/env python3
"""Send one Nav2 goal and record all three omnidirectional velocity axes."""

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
from geometry_msgs.msg import Twist
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


class Recorder(Node):
    def __init__(self) -> None:
        super().__init__("b2_omni_mppi_trial")
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose: tuple[float, float, float] | None = None
        self.start_pose: tuple[float, float, float] | None = None
        self.odom_velocity = (0.0, 0.0, 0.0)
        self.raw_cmd = (0.0, 0.0, 0.0)
        self.smooth_cmd = (0.0, 0.0, 0.0)
        self.minimum_scan = math.nan
        self.plan: np.ndarray | None = None
        self.last_odom_xy: tuple[float, float] | None = None
        self.odom_path_length = 0.0
        self.active = False
        self.start_wall = math.nan
        self.rows: list[dict[str, float]] = []
        self.raw_samples: list[tuple[float, float, float, float]] = []
        self.smooth_samples: list[tuple[float, float, float, float]] = []
        self.scan_samples: list[float] = []
        self.cross_track_samples: list[float] = []

        self.create_subscription(
            Odometry, "odom", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(Twist, "cmd_vel_nav", self.on_raw, 50)
        self.create_subscription(Twist, "cmd_vel", self.on_smooth, 50)
        self.create_subscription(
            LaserScan, "scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(NavPath, "plan", self.on_plan, 10)
        self.create_timer(0.05, self.snapshot)

    def elapsed(self) -> float:
        return 0.0 if not self.active else time.monotonic() - self.start_wall

    def refresh_pose(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except TransformException:
            return False
        t = tf.transform.translation
        self.pose = (
            t.x,
            t.y,
            yaw_from_quaternion(tf.transform.rotation),
        )
        if self.active and self.plan is not None:
            point = np.asarray(self.pose[:2])
            self.cross_track_samples.append(
                float(np.min(np.linalg.norm(self.plan - point, axis=1)))
            )
        return True

    def on_odom(self, msg: Odometry) -> None:
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.active and self.last_odom_xy is not None:
            self.odom_path_length += math.hypot(
                xy[0] - self.last_odom_xy[0],
                xy[1] - self.last_odom_xy[1],
            )
        self.last_odom_xy = xy
        twist = msg.twist.twist
        self.odom_velocity = (
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        )

    def on_raw(self, msg: Twist) -> None:
        self.raw_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        if self.active:
            self.raw_samples.append((self.elapsed(), *self.raw_cmd))

    def on_smooth(self, msg: Twist) -> None:
        self.smooth_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        if self.active:
            self.smooth_samples.append((self.elapsed(), *self.smooth_cmd))

    def on_scan(self, msg: LaserScan) -> None:
        finite = [
            value
            for value in msg.ranges
            if math.isfinite(value)
            and msg.range_min <= value <= msg.range_max
        ]
        self.minimum_scan = min(finite) if finite else math.nan
        if self.active and math.isfinite(self.minimum_scan):
            self.scan_samples.append(self.minimum_scan)

    def on_plan(self, msg: NavPath) -> None:
        if msg.poses:
            self.plan = np.asarray(
                [
                    (pose.pose.position.x, pose.pose.position.y)
                    for pose in msg.poses
                ],
                dtype=float,
            )

    def snapshot(self) -> None:
        if not self.active:
            return
        self.refresh_pose()
        pose = self.pose or (math.nan, math.nan, math.nan)
        self.rows.append(
            {
                "wall_time_s": self.elapsed(),
                "map_x_m": pose[0],
                "map_y_m": pose[1],
                "map_yaw_rad": pose[2],
                "odom_vx_mps": self.odom_velocity[0],
                "odom_vy_mps": self.odom_velocity[1],
                "odom_wz_radps": self.odom_velocity[2],
                "raw_cmd_vx_mps": self.raw_cmd[0],
                "raw_cmd_vy_mps": self.raw_cmd[1],
                "raw_cmd_wz_radps": self.raw_cmd[2],
                "smoothed_cmd_vx_mps": self.smooth_cmd[0],
                "smoothed_cmd_vy_mps": self.smooth_cmd[1],
                "smoothed_cmd_wz_radps": self.smooth_cmd[2],
                "minimum_scan_range_m": self.minimum_scan,
            }
        )


def mean_rate(samples: list[tuple[float, ...]]) -> float:
    if len(samples) < 2:
        return math.nan
    duration = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / duration if duration > 0.0 else math.nan


def total_variation(
    samples: list[tuple[float, ...]], index: int
) -> float:
    return float(
        sum(
            abs(samples[i][index] - samples[i - 1][index])
            for i in range(1, len(samples))
        )
    )


def lateral_share(samples: list[tuple[float, ...]]) -> float:
    lateral = sum(abs(sample[2]) for sample in samples)
    planar = sum(abs(sample[1]) + abs(sample[2]) for sample in samples)
    return lateral / planar if planar > 0.0 else math.nan


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", choices=("custom", "official"), required=True)
    parser.add_argument("--scenario", default="lateral")
    parser.add_argument("--trial", default="smoke_01")
    parser.add_argument("--goal-x", type=float, default=-2.0)
    parser.add_argument("--goal-y", type=float, default=0.3)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(__file__).resolve().parent / "results/trials",
    )
    args = parser.parse_args()

    output_dir = (
        args.output_root / args.scenario / args.variant / args.trial
    ).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Recorder()
    status = GoalStatus.STATUS_UNKNOWN
    message = ""
    goal_handle = None
    action_duration = math.nan
    action_result_pose: tuple[float, float, float] | None = None
    settled_pose: tuple[float, float, float] | None = None
    try:
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.action.server_is_ready() and node.refresh_pose():
                break
        else:
            raise TimeoutError("Nav2 action server or map->base_link TF not ready")

        # The action server and TF can be ready before DDS discovery has
        # matched this recorder to /cmd_vel_nav and /cmd_vel. Give read-only
        # subscriptions time to match so the first acceleration ramp is not
        # missing from the recorded command trace.
        warmup_deadline = time.monotonic() + 1.5
        while time.monotonic() < warmup_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.refresh_pose()

        node.start_pose = node.pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        goal.pose.pose.orientation.z = math.sin(args.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(args.goal_yaw / 2.0)

        send_future = node.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
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
            action_duration = node.elapsed()
            node.refresh_pose()
            action_result_pose = node.pose
        else:
            message = f"timeout after {args.timeout:.1f}s"
            action_duration = node.elapsed()
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                node, cancel_future, timeout_sec=5.0
            )
        node.active = False
        # The controller stops publishing the target immediately, but the
        # external velocity smoother may need time to ramp /cmd_vel to zero.
        # Observe this separately so action time and tracking metrics remain
        # comparable while post-result drift is still measurable.
        settle_deadline = time.monotonic() + 1.5
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.refresh_pose()
        settled_pose = node.pose
    except Exception as exc:
        message = f"{type(exc).__name__}: {exc}"
    finally:
        node.refresh_pose()
        final_pose = (
            settled_pose
            or node.pose
            or (math.nan, math.nan, math.nan)
        )
        start_pose = node.start_pose or (math.nan, math.nan, math.nan)
        result_pose = (
            action_result_pose
            or final_pose
        )
        plan_endpoint = (
            tuple(node.plan[-1])
            if node.plan is not None and len(node.plan) > 0
            else (math.nan, math.nan)
        )
        summary = {
            "scenario": args.scenario,
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
            "duration_wall_s": (
                action_duration if math.isfinite(action_duration) else 0.0
            ),
            "start_pose": {
                "x": start_pose[0],
                "y": start_pose[1],
                "yaw": start_pose[2],
            },
            "final_pose": {
                "x": final_pose[0],
                "y": final_pose[1],
                "yaw": final_pose[2],
            },
            "action_result_pose": {
                "x": result_pose[0],
                "y": result_pose[1],
                "yaw": result_pose[2],
            },
            "action_result_goal_error_m": math.hypot(
                result_pose[0] - args.goal_x,
                result_pose[1] - args.goal_y,
            ),
            "plan_endpoint": {
                "x": plan_endpoint[0],
                "y": plan_endpoint[1],
            },
            "action_result_plan_endpoint_error_m": math.hypot(
                result_pose[0] - plan_endpoint[0],
                result_pose[1] - plan_endpoint[1],
            ),
            "post_result_drift_m": math.hypot(
                final_pose[0] - result_pose[0],
                final_pose[1] - result_pose[1],
            ),
            "net_displacement": {
                "x": final_pose[0] - start_pose[0],
                "y": final_pose[1] - start_pose[1],
            },
            "final_goal_error_m": math.hypot(
                final_pose[0] - args.goal_x,
                final_pose[1] - args.goal_y,
            ),
            "odom_path_length_m": node.odom_path_length,
            "minimum_scan_range_m": (
                min(node.scan_samples) if node.scan_samples else math.nan
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
            "raw_cmd_rate_hz": mean_rate(node.raw_samples),
            "smoothed_cmd_rate_hz": mean_rate(node.smooth_samples),
            "raw_lateral_command_share": lateral_share(node.raw_samples),
            "smoothed_lateral_command_share": lateral_share(
                node.smooth_samples
            ),
            "raw_cmd_total_variation": {
                "vx": total_variation(node.raw_samples, 1),
                "vy": total_variation(node.raw_samples, 2),
                "wz": total_variation(node.raw_samples, 3),
            },
            "raw_peak_abs_vy_mps": (
                max(abs(sample[2]) for sample in node.raw_samples)
                if node.raw_samples
                else math.nan
            ),
            "timeseries_rows": len(node.rows),
        }

        with (output_dir / "summary.json").open(
            "w", encoding="utf-8"
        ) as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2)

        if node.rows:
            with (output_dir / "timeseries.csv").open(
                "w", encoding="utf-8", newline=""
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

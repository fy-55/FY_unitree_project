#!/usr/bin/env python3
"""Generate fair isolated Nav2 configs without editing the real B2 project."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent
NAV_WS = ROOT.parents[1]
OFFICIAL_SOURCE = (
    NAV_WS
    / "experiments/nav2_official_mppi_sim/config"
    / "nav2_official_mppi_params.yaml"
)
B2_SOURCE = NAV_WS / "src/b2_navigation/params/dog_nav_params.yaml"
CUSTOM_CPP = (
    NAV_WS
    / "src/nav2_custom_plugins/src/mppi_gpu_controller.cpp"
)
CUSTOM_CUDA = (
    NAV_WS
    / "src/nav2_custom_plugins/src/mppi_gpu_kernels.cu"
)
CUSTOM_LIBRARY = (
    NAV_WS
    / "install/nav2_custom_plugins/lib"
    / "libnav2_custom_plugins_gpu.so"
)
MAP_YAML = Path(
    "/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml"
)
OUTPUT_DIR = ROOT / "config/generated"
RUNTIME_DIR = ROOT / "results/runtime"


def read_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def write_yaml(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            data,
            stream,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def configure_costmap(node: dict, *, rolling: bool) -> None:
    params = node["ros__parameters"]
    params["use_sim_time"] = True
    params["robot_base_frame"] = "base_link"
    params["footprint"] = (
        "[[0.50, 0.30], [0.50, -0.30], "
        "[-0.50, -0.30], [-0.50, 0.30]]"
    )
    params.pop("robot_radius", None)
    params["resolution"] = 0.05
    params["plugins"] = (
        ["obstacle_layer", "inflation_layer"]
        if rolling
        else ["static_layer", "obstacle_layer", "inflation_layer"]
    )
    params.pop("voxel_layer", None)
    params["obstacle_layer"] = {
        "plugin": "nav2_costmap_2d::ObstacleLayer",
        "enabled": True,
        "observation_sources": "scan",
        "scan": {
            "topic": "/scan",
            "max_obstacle_height": 2.0,
            "clearing": True,
            "marking": True,
            "data_type": "LaserScan",
            "raytrace_max_range": 6.0,
            "raytrace_min_range": 0.1,
            "obstacle_max_range": 5.5,
            "obstacle_min_range": 0.1,
        },
    }
    params["inflation_layer"] = {
        "plugin": "nav2_costmap_2d::InflationLayer",
        "cost_scaling_factor": 3.0,
        "inflation_radius": 0.75,
    }
    if rolling:
        params["global_frame"] = "odom"
        params["rolling_window"] = True
        params["width"] = 6
        params["height"] = 6
        params.pop("static_layer", None)
    else:
        params["global_frame"] = "map"
        params["track_unknown_space"] = True
        params["static_layer"] = {
            "plugin": "nav2_costmap_2d::StaticLayer",
            "map_subscribe_transient_local": True,
        }


def configure_common(data: dict, b2: dict) -> None:
    controller = data["controller_server"]["ros__parameters"]
    controller["use_sim_time"] = True
    controller["controller_frequency"] = 20.0
    controller["min_x_velocity_threshold"] = 0.001
    controller["min_y_velocity_threshold"] = 0.001
    controller["min_theta_velocity_threshold"] = 0.001
    controller["failure_tolerance"] = 0.3
    # Keep the real project's Nav2 completion/progress semantics. The first
    # strict smoke used 0.20 m and showed the custom controller parked at
    # 0.251 m; the deployed project actually declares 0.35 m and 0.60 rad.
    b2_controller = b2["controller_server"]["ros__parameters"]
    controller["progress_checker"] = copy.deepcopy(
        b2_controller["progress_checker"]
    )
    controller["general_goal_checker"] = copy.deepcopy(
        b2_controller["general_goal_checker"]
    )

    configure_costmap(data["local_costmap"]["local_costmap"], rolling=True)
    configure_costmap(data["global_costmap"]["global_costmap"], rolling=False)

    data["map_server"]["ros__parameters"]["use_sim_time"] = True
    data["map_server"]["ros__parameters"]["yaml_filename"] = str(MAP_YAML)

    planner = data["planner_server"]["ros__parameters"]
    planner["use_sim_time"] = True
    planner["GridBased"]["tolerance"] = 0.20

    bt = data["bt_navigator"]["ros__parameters"]
    bt["use_sim_time"] = True
    bt["global_frame"] = "map"
    bt["robot_base_frame"] = "base_link"
    bt["odom_topic"] = "/odom"

    behavior = data["behavior_server"]["ros__parameters"]
    behavior["use_sim_time"] = True
    behavior["global_frame"] = "odom"
    behavior["robot_base_frame"] = "base_link"

    # Preserve the real project's external smoother settings in both groups.
    data["velocity_smoother"] = copy.deepcopy(b2["velocity_smoother"])
    data["velocity_smoother"]["ros__parameters"]["use_sim_time"] = True

    for key in (
        "smoother_server",
        "waypoint_follower",
        "bt_navigator_navigate_through_poses_rclcpp_node",
        "bt_navigator_navigate_to_pose_rclcpp_node",
    ):
        if key in data:
            data[key]["ros__parameters"]["use_sim_time"] = True


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)

    official_source = read_yaml(OFFICIAL_SOURCE)
    b2 = read_yaml(B2_SOURCE)

    official = copy.deepcopy(official_source)
    custom = copy.deepcopy(official_source)
    configure_common(official, b2)
    configure_common(custom, b2)

    official_follow = official["controller_server"]["ros__parameters"][
        "FollowPath"
    ]
    official_follow.update(
        {
            "plugin": "nav2_mppi_controller::MPPIController",
            "motion_model": "Omni",
            "vx_max": 0.80,
            "vx_min": -0.15,
            "vy_max": 0.30,
            "wz_max": 0.70,
            "vx_std": 0.50,
            "vy_std": 0.30,
            "wz_std": 0.40,
            "visualize": False,
        }
    )
    if "CostCritic" in official_follow:
        official_follow["CostCritic"]["consider_footprint"] = True

    custom_follow = copy.deepcopy(
        b2["controller_server"]["ros__parameters"]["FollowPath"]
    )
    custom_follow.update(
        {
            "plugin": "nav2_custom_plugins/MPPIGPUController",
            "enable_stats": True,
            "stats_file_path": str(
                RUNTIME_DIR / "custom_gpu_controller_stats.csv"
            ),
            "enable_file_log": True,
            "log_file_path": str(
                RUNTIME_DIR / "custom_gpu_controller.log"
            ),
        }
    )
    custom["controller_server"]["ros__parameters"][
        "FollowPath"
    ] = custom_follow

    custom_path = OUTPUT_DIR / "custom_gpu_omni.yaml"
    official_path = OUTPUT_DIR / "official_omni.yaml"
    write_yaml(custom_path, custom)
    write_yaml(official_path, official)

    snapshot = {
        "scope": (
            "Ideal-response B2-sized omnidirectional 2D Gazebo plant. "
            "It is not leg dynamics or real-B2 evidence."
        ),
        "shared_conditions": {
            "controller_frequency_hz": 20.0,
            "body_size_m": [1.0, 0.6],
            "velocity_smoother": data_or_none(
                b2, "velocity_smoother", "ros__parameters"
            ),
            "map": str(MAP_YAML),
        },
        "custom_follow_path": custom_follow,
        "official_follow_path": official_follow,
        "source_sha256": {
            str(path): sha256(path)
            for path in (
                OFFICIAL_SOURCE,
                B2_SOURCE,
                CUSTOM_CPP,
                CUSTOM_CUDA,
                CUSTOM_LIBRARY,
                ROOT / "worlds/b2_omni_world.world",
            )
        },
    }
    snapshot_path = OUTPUT_DIR / "config_snapshot.json"
    with snapshot_path.open("w", encoding="utf-8") as stream:
        json.dump(snapshot, stream, ensure_ascii=False, indent=2)

    print(f"Generated: {custom_path}")
    print(f"Generated: {official_path}")
    print(f"Snapshot:  {snapshot_path}")


def data_or_none(data: dict, *keys: str):
    value = data
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


if __name__ == "__main__":
    main()

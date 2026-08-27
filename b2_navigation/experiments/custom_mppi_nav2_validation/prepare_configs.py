#!/usr/bin/env python3
"""Generate isolated Nav2 configs without editing the real navigation project."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import yaml


EXPERIMENT_ROOT = Path(__file__).resolve().parent
NAV_WS = EXPERIMENT_ROOT.parents[1]
OFFICIAL_SOURCE = (
    NAV_WS
    / "experiments/nav2_official_mppi_sim/config"
    / "nav2_official_mppi_params.yaml"
)
REAL_B2_PARAMS = NAV_WS / "src/b2_navigation/params/dog_nav_params.yaml"
CUSTOM_CONTROLLER = (
    NAV_WS
    / "src/nav2_custom_plugins/src"
    / "mppi_gpu_controller.cpp"
)
CUSTOM_KERNEL = (
    NAV_WS / "src/nav2_custom_plugins/src/mppi_gpu_kernels.cu"
)
CUSTOM_LIBRARY = (
    NAV_WS
    / "build/nav2_custom_plugins"
    / "libnav2_custom_plugins_gpu.so"
)
OUTPUT_DIR = EXPERIMENT_ROOT / "config/generated"
RUNTIME_DIR = EXPERIMENT_ROOT / "results/runtime"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)

    official = read_yaml(OFFICIAL_SOURCE)
    b2 = read_yaml(REAL_B2_PARAMS)

    official_reference = copy.deepcopy(official)
    official_follow = official_reference["controller_server"][
        "ros__parameters"
    ]["FollowPath"]
    # Disable trajectory visualization for timing and automatic trials.
    official_follow["visualize"] = False

    custom_gpu = copy.deepcopy(official)
    custom_follow = copy.deepcopy(
        b2["controller_server"]["ros__parameters"]["FollowPath"]
    )

    # Only robot-compatibility adaptations are made here.  The core custom
    # choices remain N=8000, H=5, dt=0.1, mixed noise, current cost ratios,
    # warm start and GPU rollout.  TurtleBot3 Waffle is differential drive and
    # much slower/smaller than B2, so lateral motion, speed and footprint must
    # be adapted before it is safe or meaningful to run in this simulator.
    adaptations = {
        "plugin": "nav2_custom_plugins/MPPIGPUController",
        "max_v": 0.26,
        "min_v": -0.15,
        "max_vy": 0.0,
        "max_w": 1.0,
        "action_std_v": 0.20,
        "action_std_vy": 0.0,
        "action_std_w": 0.40,
        "footprint_front": 0.22,
        "footprint_back": 0.22,
        "footprint_left": 0.22,
        "footprint_right": 0.22,
        "enable_lateral_bias": False,
        "enable_narrow_passage": False,
        "enable_stats": True,
        "stats_file_path": str(
            RUNTIME_DIR / "custom_gpu_controller_stats.csv"
        ),
        "enable_file_log": True,
        "log_file_path": str(
            RUNTIME_DIR / "custom_gpu_controller.log"
        ),
    }
    custom_follow.update(adaptations)
    custom_gpu["controller_server"]["ros__parameters"][
        "FollowPath"
    ] = custom_follow

    official_path = OUTPUT_DIR / "official_reference.yaml"
    custom_path = OUTPUT_DIR / "custom_gpu_tb3.yaml"
    write_yaml(official_path, official_reference)
    write_yaml(custom_path, custom_gpu)

    snapshot = {
        "scope": (
            "TurtleBot3 Gazebo compatibility test. It does not start B2 "
            "nodes and is not evidence of real B2 performance."
        ),
        "shared_stack": (
            "Both variants use the same Nav2, map, planner, costmaps, goal "
            "checker, velocity smoother, TurtleBot3 model, start, and goal."
        ),
        "official_follow_path": official_follow,
        "custom_follow_path": custom_follow,
        "custom_tb3_adaptations": adaptations,
        "source_sha256": {
            str(path): sha256(path)
            for path in (
                OFFICIAL_SOURCE,
                REAL_B2_PARAMS,
                CUSTOM_CONTROLLER,
                CUSTOM_KERNEL,
                CUSTOM_LIBRARY,
            )
        },
    }
    with (OUTPUT_DIR / "config_snapshot.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(snapshot, stream, ensure_ascii=False, indent=2)

    print(f"Generated: {official_path}")
    print(f"Generated: {custom_path}")
    print(f"Snapshot:  {OUTPUT_DIR / 'config_snapshot.json'}")


if __name__ == "__main__":
    main()

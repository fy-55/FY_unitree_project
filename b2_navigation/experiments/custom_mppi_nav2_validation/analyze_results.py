#!/usr/bin/env python3
"""Aggregate isolated smoke trials and generate an evidence overview."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
TRIAL_ROOT = ROOT / "results/trials"
OUTPUT_DIR = ROOT / "results"


def configure_plot_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": [
                "Noto Sans CJK JP",
                "Droid Sans Fallback",
                "DejaVu Sans",
            ],
            "axes.unicode_minus": False,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def scenario(summary: dict) -> str:
    goal = summary["goal"]
    if abs(goal["x"] + 1.0) < 1e-9 and abs(goal["y"] + 0.5) < 1e-9:
        return "straight"
    if abs(goal["x"] - 1.5) < 1e-9 and abs(goal["y"] - 0.5) < 1e-9:
        return "turning"
    return "other"


def load_valid_trials() -> list[dict]:
    rows: list[dict] = []
    for path in sorted(TRIAL_ROOT.glob("*/*/summary.json")):
        with path.open(encoding="utf-8") as stream:
            summary = json.load(stream)
        if int(summary["timeseries_rows"]) <= 0:
            continue
        summary["_summary_path"] = path
        summary["_timeseries_path"] = path.with_name("timeseries.csv")
        summary["_scenario"] = scenario(summary)
        rows.append(summary)
    return rows


def load_timeseries(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    keys = rows[0].keys()
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=float)
        for key in keys
    }


def flat_row(summary: dict) -> dict[str, object]:
    duration = float(summary["duration_wall_s"])
    return {
        "scenario": summary["_scenario"],
        "variant": summary["variant"],
        "trial": summary["trial"],
        "succeeded": int(summary["succeeded"]),
        "duration_wall_s": duration,
        "final_goal_error_m": summary["final_goal_error_m"],
        "odom_path_length_m": summary["odom_path_length_m"],
        "approx_minimum_body_clearance_m": summary[
            "approx_minimum_body_clearance_m"
        ],
        "mean_cross_track_error_m": summary["mean_cross_track_error_m"],
        "p95_cross_track_error_m": summary["p95_cross_track_error_m"],
        "raw_cmd_rate_hz": summary["raw_cmd_rate_hz"],
        "smoothed_cmd_rate_hz": summary["smoothed_cmd_rate_hz"],
        "raw_vx_total_variation": summary["raw_vx_total_variation"],
        "raw_wz_total_variation": summary["raw_wz_total_variation"],
        "raw_vx_variation_per_s": (
            float(summary["raw_vx_total_variation"]) / duration
        ),
        "raw_wz_variation_per_s": (
            float(summary["raw_wz_total_variation"]) / duration
        ),
    }


def select(
    trials: list[dict], variant: str, scenario_name: str
) -> dict:
    return next(
        row
        for row in trials
        if row["variant"] == variant
        and row["_scenario"] == scenario_name
    )


def generate_plot(trials: list[dict]) -> None:
    custom_turn = select(trials, "custom", "turning")
    official_turn = select(trials, "official", "turning")
    custom_straight = select(trials, "custom", "straight")
    data = {
        "custom_turn": load_timeseries(custom_turn["_timeseries_path"]),
        "official_turn": load_timeseries(official_turn["_timeseries_path"]),
        "custom_straight": load_timeseries(
            custom_straight["_timeseries_path"]
        ),
    }

    colors = {
        "custom_turn": "#d94841",
        "official_turn": "#2878b5",
        "custom_straight": "#2f855a",
    }
    labels = {
        "custom_turn": "自定义GPU-MPPI：转弯目标（超时）",
        "official_turn": "官方MPPI：转弯目标（成功）",
        "custom_straight": "自定义GPU-MPPI：直线目标（成功）",
    }

    figure, axes = plt.subplots(2, 2, figsize=(13.2, 9.2))
    ax = axes[0, 0]
    for key in ("custom_turn", "official_turn", "custom_straight"):
        ax.plot(
            data[key]["amcl_x_m"],
            data[key]["amcl_y_m"],
            color=colors[key],
            linewidth=2.0,
            label=labels[key],
        )
    ax.scatter([-2.0], [-0.5], marker="o", s=55, color="#222222", label="起点")
    ax.scatter(
        [1.5, -1.0],
        [0.5, -0.5],
        marker="*",
        s=130,
        color=["#805ad5", "#38a169"],
        label="两个目标",
    )
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title("二维轨迹：直线可用，明显转向失败")
    ax.axis("equal")
    ax.legend(fontsize=8)

    for key in ("custom_turn", "official_turn"):
        mask = data[key]["wall_time_s"] <= 30.0
        axes[0, 1].plot(
            data[key]["wall_time_s"][mask],
            data[key]["raw_cmd_vx_mps"][mask],
            color=colors[key],
            linewidth=1.6,
            label=labels[key],
        )
        axes[1, 0].plot(
            data[key]["wall_time_s"][mask],
            data[key]["raw_cmd_wz_radps"][mask],
            color=colors[key],
            linewidth=1.4,
            label=labels[key],
        )
    axes[0, 1].set_xlabel("时间 (s)")
    axes[0, 1].set_ylabel("MPPI原始 vx (m/s)")
    axes[0, 1].set_title("转弯目标：前30秒线速度指令")
    axes[0, 1].legend(fontsize=8)
    axes[1, 0].set_xlabel("时间 (s)")
    axes[1, 0].set_ylabel("MPPI原始 wz (rad/s)")
    axes[1, 0].set_title("转弯目标：前30秒角速度指令")
    axes[1, 0].legend(fontsize=8)

    axes[1, 1].axis("off")
    table_rows = []
    for label, row in (
        ("自定义/转弯", custom_turn),
        ("官方/转弯", official_turn),
        ("自定义/直线", custom_straight),
    ):
        duration = float(row["duration_wall_s"])
        table_rows.append(
            [
                label,
                "成功" if row["succeeded"] else "失败",
                f"{duration:.2f}",
                f"{float(row['final_goal_error_m']):.3f}",
                f"{float(row['raw_cmd_rate_hz']):.2f}",
                f"{float(row['raw_wz_total_variation']) / duration:.2f}",
            ]
        )
    table = axes[1, 1].table(
        cellText=table_rows,
        colLabels=[
            "试验",
            "结果",
            "耗时(s)",
            "终点误差(m)",
            "原始指令Hz",
            "角速度变化/s",
        ],
        loc="center",
        cellLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.5)
    table.scale(1.1, 1.7)
    axes[1, 1].set_title("冒烟试验指标（单次，不是统计结论）", pad=15)

    figure.suptitle(
        "自定义GPU-MPPI隔离Nav2/Gazebo初步验证\n"
        "TurtleBot3差速兼容测试，不代表真实B2性能",
        fontsize=14,
    )
    figure.tight_layout()
    figure.savefig(
        OUTPUT_DIR / "validation_overview.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    configure_plot_style()
    trials = load_valid_trials()
    flat = [flat_row(row) for row in trials]
    with (OUTPUT_DIR / "trial_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0].keys()))
        writer.writeheader()
        writer.writerows(flat)
    generate_plot(trials)
    print(f"Wrote: {OUTPUT_DIR / 'trial_summary.csv'}")
    print(f"Wrote: {OUTPUT_DIR / 'validation_overview.png'}")


if __name__ == "__main__":
    main()

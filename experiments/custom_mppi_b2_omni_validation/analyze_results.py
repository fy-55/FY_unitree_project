#!/usr/bin/env python3
"""Summarize the formal reachable lateral comparison and draw evidence plots."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import Circle


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
TRIAL_ROOT = RESULTS / "trials/lateral_free"
FORMAL = {
    "custom": TRIAL_ROOT / "custom/formal_02",
    "official": TRIAL_ROOT / "official/formal_02",
}


def set_chinese_font() -> None:
    candidates = (
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "WenQuanYi Micro Hei",
        "DejaVu Sans",
    )
    available = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in candidates:
        if candidate in available:
            plt.rcParams["font.sans-serif"] = [candidate]
            break
    plt.rcParams["axes.unicode_minus"] = False


def read_summary(path: Path) -> dict:
    with (path / "summary.json").open(encoding="utf-8") as stream:
        return json.load(stream)


def read_series(path: Path) -> np.ndarray:
    return np.genfromtxt(
        path / "timeseries.csv",
        delimiter=",",
        names=True,
        encoding="utf-8",
    )


def main() -> None:
    set_chinese_font()
    summaries = {key: read_summary(path) for key, path in FORMAL.items()}
    series = {key: read_series(path) for key, path in FORMAL.items()}

    fields = [
        "variant",
        "succeeded",
        "duration_wall_s",
        "action_result_goal_error_m",
        "final_goal_error_m",
        "post_result_drift_m",
        "odom_path_length_m",
        "smoothed_lateral_command_share",
        "raw_cmd_rate_hz",
        "minimum_scan_range_m",
        "mean_cross_track_error_m",
        "p95_cross_track_error_m",
    ]
    csv_path = RESULTS / "formal_summary.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for variant in ("custom", "official"):
            row = {"variant": variant}
            row.update(
                {
                    field: summaries[variant][field]
                    for field in fields
                    if field != "variant"
                }
            )
            writer.writerow(row)

    colors = {"custom": "#d94b41", "official": "#2878b5"}
    labels = {
        "custom": "自定义 GPU-MPPI",
        "official": "官方 Omni MPPI",
    }
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(
        "B2式全向二维理想底盘：0.8 m纯横移单次冒烟对照\n"
        "共同保留外部velocity_smoother；不代表真实B2或统计结论",
        fontsize=18,
    )

    ax = axes[0, 0]
    for variant in ("custom", "official"):
        data = series[variant]
        summary = summaries[variant]
        ax.plot(
            data["map_x_m"],
            data["map_y_m"],
            color=colors[variant],
            linewidth=2.5,
            label=labels[variant],
        )
        ax.scatter(
            summary["action_result_pose"]["x"],
            summary["action_result_pose"]["y"],
            color=colors[variant],
            marker="o",
            s=80,
        )
        ax.scatter(
            summary["final_pose"]["x"],
            summary["final_pose"]["y"],
            color=colors[variant],
            marker="x",
            s=90,
        )
    goal = summaries["custom"]["goal"]
    ax.scatter(goal["x"], goal["y"], marker="*", s=220, color="#7755cc")
    ax.add_patch(
        Circle(
            (goal["x"], goal["y"]),
            0.35,
            fill=False,
            linestyle="--",
            linewidth=1.5,
            color="#7755cc",
            label="当前项目0.35 m到点容差",
        )
    )
    ax.set_title("二维轨迹（圆点=动作成功时，叉号=1.5秒后）")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")

    for col, variant in enumerate(("custom", "official")):
        ax = axes[0, 1] if col == 0 else axes[1, 0]
        data = series[variant]
        time = data["wall_time_s"]
        ax.plot(
            time,
            data["raw_cmd_vy_mps"],
            color="#999999",
            linestyle="--",
            linewidth=1.2,
            label="MPPI原始 vy",
        )
        ax.plot(
            time,
            data["smoothed_cmd_vy_mps"],
            color=colors[variant],
            linewidth=2.2,
            label="平滑后 vy",
        )
        ax.plot(
            time,
            data["odom_vy_mps"],
            color="#2a9d65",
            linewidth=1.4,
            alpha=0.85,
            label="里程计 vy",
        )
        ax.set_title(f"{labels[variant]}：横向速度调用链")
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("vy (m/s)")
        ax.grid(alpha=0.3)
        ax.legend(loc="best")

    ax = axes[1, 1]
    ax.axis("off")
    rows = []
    for variant in ("custom", "official"):
        summary = summaries[variant]
        rows.append(
            [
                labels[variant],
                "成功" if summary["succeeded"] else "失败",
                f'{summary["duration_wall_s"]:.2f}',
                f'{summary["action_result_goal_error_m"]:.3f}',
                f'{summary["post_result_drift_m"]:.3f}',
                f'{100 * summary["smoothed_lateral_command_share"]:.1f}%',
                f'{summary["raw_cmd_rate_hz"]:.2f}',
            ]
        )
    table = ax.table(
        cellText=rows,
        colLabels=[
            "控制器",
            "结果",
            "耗时(s)",
            "成功时误差(m)",
            "成功后漂移(m)",
            "平滑后vy占比",
            "指令Hz",
        ],
        loc="center",
        cellLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.05, 2.0)
    ax.set_title("正式可达目标冒烟指标（单次）", pad=15)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    plot_path = RESULTS / "omni_validation_overview.png"
    fig.savefig(plot_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    print(f"Wrote: {csv_path}")
    print(f"Wrote: {plot_path}")


if __name__ == "__main__":
    main()

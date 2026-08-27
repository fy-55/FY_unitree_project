#!/usr/bin/env python3
"""Offline sensitivity study for the B2 GPU-MPPI motion-model assumption.

This is deliberately a small, transparent experiment.  It does not claim to
model the real Unitree B2 before real command/odometry data are collected.

The current GPU rollout treats sampled body velocities as immediately realised
velocities.  Here we compare that ideal assumption with two hypothetical plants
that include command delay, first-order velocity lag, and a steady-state scale.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import matplotlib.pyplot as plt
import numpy as np


CONTROL_PERIOD = 0.05
MODEL_DT = 0.1
HORIZON_STEPS = 5
HORIZON_SECONDS = MODEL_DT * HORIZON_STEPS
TEST_COMMAND = 0.4
CLEARANCE_THRESHOLD = 0.15


@dataclass(frozen=True)
class Plant:
    key: str
    label: str
    tau: float
    delay: float
    scale: float
    color: str


PLANTS = (
    Plant("ideal", "理想模型", tau=0.0, delay=0.0, scale=1.0, color="#1f77b4"),
    Plant(
        "mild",
        "轻度失配：延迟0.05s，τ=0.15s，比例0.85",
        tau=0.15,
        delay=0.05,
        scale=0.85,
        color="#f39c12",
    ),
    Plant(
        "severe",
        "严重失配：延迟0.10s，τ=0.30s，比例0.50",
        tau=0.30,
        delay=0.10,
        scale=0.50,
        color="#c0392b",
    ),
)


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


def simulate(
    plant: Plant,
    command: Callable[[float], float],
    duration: float,
    integration_dt: float = 0.001,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Simulate a one-dimensional velocity response and integrated position."""
    times = np.arange(0.0, duration + integration_dt * 0.5, integration_dt)
    command_now = np.array([command(float(t)) for t in times])
    velocity = np.zeros_like(times)
    position = np.zeros_like(times)

    for i in range(1, len(times)):
        previous_t = float(times[i - 1])
        delayed_t = previous_t - plant.delay
        delayed_command = command(delayed_t) if delayed_t >= 0.0 else 0.0
        target_velocity = plant.scale * delayed_command

        if plant.tau <= 0.0:
            new_velocity = target_velocity
            displacement = new_velocity * integration_dt
        else:
            decay = math.exp(-integration_dt / plant.tau)
            new_velocity = target_velocity + (velocity[i - 1] - target_velocity) * decay
            displacement = 0.5 * (
                velocity[i - 1] + new_velocity
            ) * integration_dt

        velocity[i] = new_velocity
        position[i] = position[i - 1] + displacement

    return times, command_now, velocity, position


def constant_command(value: float) -> Callable[[float], float]:
    return lambda _t: value


def pulse_command(t: float) -> float:
    if 0.10 <= t < 0.70:
        return TEST_COMMAND
    return 0.0


def value_at(times: np.ndarray, values: np.ndarray, target_time: float) -> float:
    return float(np.interp(target_time, times, values))


def save_csv(
    output_dir: Path,
    horizon_results: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
) -> None:
    summary_path = output_dir / "summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "scenario",
                "tau_s",
                "delay_s",
                "velocity_scale",
                "command_mps",
                "predicted_y_0p1_m",
                "actual_y_0p1_m",
                "predicted_y_0p5_m",
                "actual_y_0p5_m",
                "horizon_error_m",
                "actual_over_predicted",
                "predicted_safe_at_0p15m",
                "actual_safe_at_0p15m",
            ]
        )
        predicted_01 = TEST_COMMAND * 0.1
        predicted_05 = TEST_COMMAND * HORIZON_SECONDS
        for plant in PLANTS:
            times, _command, _velocity, position = horizon_results[plant.key]
            actual_01 = value_at(times, position, 0.1)
            actual_05 = value_at(times, position, HORIZON_SECONDS)
            writer.writerow(
                [
                    plant.key,
                    f"{plant.tau:.3f}",
                    f"{plant.delay:.3f}",
                    f"{plant.scale:.3f}",
                    f"{TEST_COMMAND:.3f}",
                    f"{predicted_01:.6f}",
                    f"{actual_01:.6f}",
                    f"{predicted_05:.6f}",
                    f"{actual_05:.6f}",
                    f"{actual_05 - predicted_05:.6f}",
                    f"{actual_05 / predicted_05:.6f}",
                    int(predicted_05 >= CLEARANCE_THRESHOLD),
                    int(actual_05 >= CLEARANCE_THRESHOLD),
                ]
            )

    candidates_path = output_dir / "candidate_clearance.csv"
    candidate_commands = np.array([0.2, 0.3, 0.4, 0.5])
    with candidates_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "command_mps",
                "predicted_displacement_m",
                *[f"{plant.key}_actual_displacement_m" for plant in PLANTS],
            ]
        )
        for command_value in candidate_commands:
            row: list[str] = [
                f"{command_value:.3f}",
                f"{command_value * HORIZON_SECONDS:.6f}",
            ]
            for plant in PLANTS:
                times, _cmd, _velocity, position = simulate(
                    plant,
                    constant_command(float(command_value)),
                    HORIZON_SECONDS,
                )
                row.append(f"{value_at(times, position, HORIZON_SECONDS):.6f}")
            writer.writerow(row)


def plot_velocity_response(output_dir: Path) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(10.5, 6.8), sharex=True)
    for plant in PLANTS:
        times, command, velocity, position = simulate(plant, pulse_command, 1.2)
        axes[0].plot(times, velocity, color=plant.color, linewidth=2.1, label=plant.label)
        axes[1].plot(times, position, color=plant.color, linewidth=2.1, label=plant.label)

    axes[0].plot(
        times,
        command,
        color="#222222",
        linestyle="--",
        linewidth=1.6,
        label="下发速度命令",
    )
    axes[0].set_ylabel("横向速度 vy (m/s)")
    axes[0].set_title("同一速度命令下，不同执行模型的短期响应")
    axes[0].legend(loc="upper right", fontsize=9)
    axes[1].set_ylabel("横向位移 y (m)")
    axes[1].set_xlabel("时间 (s)")
    axes[1].axvline(0.10, color="#777777", linestyle=":", linewidth=1.2)
    axes[1].axvline(0.70, color="#777777", linestyle=":", linewidth=1.2)
    axes[1].legend(loc="upper left", fontsize=9)
    figure.tight_layout()
    figure.savefig(output_dir / "velocity_response.png", dpi=180, bbox_inches="tight")
    plt.close(figure)


def plot_horizon_displacement(
    output_dir: Path,
    horizon_results: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
) -> None:
    sample_times = np.arange(1, HORIZON_STEPS + 1) * MODEL_DT
    predicted = TEST_COMMAND * sample_times

    figure, axis = plt.subplots(figsize=(10.5, 5.8))
    axis.plot(
        sample_times,
        predicted,
        marker="o",
        markersize=7,
        linewidth=2.5,
        color="#222222",
        linestyle="--",
        label="GPU-MPPI理想运动学预测",
    )
    for plant in PLANTS:
        times, _command, _velocity, position = horizon_results[plant.key]
        actual = [value_at(times, position, float(t)) for t in sample_times]
        axis.plot(
            sample_times,
            actual,
            marker="o",
            markersize=6,
            linewidth=2.1,
            color=plant.color,
            label=plant.label,
        )

    axis.set_xlabel("预测时域时间 (s)")
    axis.set_ylabel("累计横向位移 (m)")
    axis.set_title(
        "H=5、dt=0.1s、vy=0.4m/s：理想预测与假设执行模型的位移差异"
    )
    axis.set_xticks(sample_times)
    axis.legend(loc="upper left", fontsize=9)
    figure.tight_layout()
    figure.savefig(output_dir / "horizon_displacement.png", dpi=180, bbox_inches="tight")
    plt.close(figure)


def plot_clearance_classification(output_dir: Path) -> None:
    candidate_commands = np.array([0.2, 0.3, 0.4, 0.5])
    predicted = candidate_commands * HORIZON_SECONDS
    scenario_values: list[np.ndarray] = []

    for plant in PLANTS:
        displacements = []
        for command_value in candidate_commands:
            times, _command, _velocity, position = simulate(
                plant,
                constant_command(float(command_value)),
                HORIZON_SECONDS,
            )
            displacements.append(value_at(times, position, HORIZON_SECONDS))
        scenario_values.append(np.array(displacements))

    figure, axis = plt.subplots(figsize=(10.5, 5.8))
    width = 0.18
    x_positions = np.arange(len(candidate_commands))
    axis.bar(
        x_positions - 1.5 * width,
        predicted,
        width,
        color="#222222",
        alpha=0.86,
        label="MPPI理想预测",
    )
    for index, (plant, values) in enumerate(zip(PLANTS, scenario_values)):
        axis.bar(
            x_positions + (index - 0.5) * width,
            values,
            width,
            color=plant.color,
            alpha=0.90,
            label=plant.label,
        )

    axis.axhline(
        CLEARANCE_THRESHOLD,
        color="#8e44ad",
        linestyle="--",
        linewidth=2.0,
        label=f"简化净空要求 {CLEARANCE_THRESHOLD:.2f}m",
    )
    axis.set_xticks(x_positions)
    axis.set_xticklabels([f"{value:.1f}" for value in candidate_commands])
    axis.set_xlabel("候选横向速度 vy (m/s)")
    axis.set_ylabel("0.5s累计横向位移 (m)")
    axis.set_title("简化净空判据：模型失配可能把“不安全”误判成“安全”")
    axis.legend(loc="upper left", fontsize=8.5, ncol=2)
    figure.tight_layout()
    figure.savefig(output_dir / "clearance_classification.png", dpi=180, bbox_inches="tight")
    plt.close(figure)


def plot_timing_chain(output_dir: Path) -> None:
    figure, axis = plt.subplots(figsize=(10.5, 4.5))
    labels = [
        "控制器重算周期",
        "速度平滑周期",
        "MPPI模型单步dt",
        "局部代价地图更新周期",
        "MPPI总预测时域",
    ]
    values = [0.05, 0.05, 0.10, 0.20, 0.50]
    colors = ["#2e86de", "#48c9b0", "#f39c12", "#af7ac5", "#34495e"]
    bars = axis.barh(labels, values, color=colors, alpha=0.92)
    axis.invert_yaxis()
    axis.set_xlabel("时间尺度 (s)")
    axis.set_title("当前导航配置中的五个时间尺度")
    for bar, value in zip(bars, values):
        axis.text(
            value + 0.008,
            bar.get_y() + bar.get_height() / 2,
            f"{value:.2f}s",
            va="center",
            fontsize=10,
        )
    axis.set_xlim(0.0, 0.57)
    figure.tight_layout()
    figure.savefig(output_dir / "timing_chain.png", dpi=180, bbox_inches="tight")
    plt.close(figure)


def print_summary(
    horizon_results: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
) -> None:
    predicted = TEST_COMMAND * HORIZON_SECONDS
    print(
        f"GPU-MPPI ideal displacement: vy={TEST_COMMAND:.2f} m/s, "
        f"H={HORIZON_STEPS}, dt={MODEL_DT:.2f} s -> {predicted:.4f} m"
    )
    for plant in PLANTS:
        times, _command, _velocity, position = horizon_results[plant.key]
        actual = value_at(times, position, HORIZON_SECONDS)
        ratio = actual / predicted if predicted else float("nan")
        print(
            f"{plant.key:>7}: displacement={actual:.4f} m, "
            f"actual/predicted={ratio:.1%}, "
            f"clearance={'PASS' if actual >= CLEARANCE_THRESHOLD else 'FAIL'}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "results",
    )
    args = parser.parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    configure_plot_style()
    horizon_results = {
        plant.key: simulate(
            plant,
            constant_command(TEST_COMMAND),
            HORIZON_SECONDS,
        )
        for plant in PLANTS
    }

    save_csv(output_dir, horizon_results)
    plot_velocity_response(output_dir)
    plot_horizon_displacement(output_dir, horizon_results)
    plot_clearance_classification(output_dir)
    plot_timing_chain(output_dir)
    print_summary(horizon_results)
    print(f"Results written to: {output_dir}")


if __name__ == "__main__":
    main()

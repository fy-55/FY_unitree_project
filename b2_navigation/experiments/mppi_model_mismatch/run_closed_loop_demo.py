#!/usr/bin/env python3
"""Minimal closed-loop MPPI model-mismatch demonstration.

This is a standalone, one-dimensional stress test.  It intentionally keeps the
same key timing/sample settings as the inspected GPU-MPPI configuration:

* controller frequency: 20 Hz
* samples per cycle: 8000
* prediction steps: 5
* model dt: 0.1 s
* MPPI temperature: 4.0

The simulated plant has an artificial 0.10 s command delay and a first-order
velocity response with tau=0.45 s.  These values are test conditions, not
measurements of a Unitree B2.

Two controllers drive the exact same plant:

1. ideal-model MPPI: sampled velocity is integrated directly as motion;
2. response-aware MPPI: the rollout includes the same artificial delay/lag.

The purpose is to show a causal, reproducible phenomenon and prepare the data
pipeline.  It is not a Nav2/Gazebo run and is not real-robot evidence.
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


CONTROL_DT = 0.05
MODEL_DT = 0.10
HORIZON_STEPS = 5
NUM_SAMPLES = 8000
TEMPERATURE = 4.0

MAX_SPEED = 0.60
GOAL_X = 0.78
WALL_X = 1.00
SAFETY_BOUNDARY_X = 0.84

# Artificial stress-test plant.  These are not B2 measurements.
PLANT_DELAY = 0.10
PLANT_TAU = 0.45
DELAY_STEPS = round(PLANT_DELAY / CONTROL_DT)
SIMULATION_SECONDS = 7.0


@dataclass
class RunResult:
    key: str
    label: str
    color: str
    time_s: np.ndarray
    position_m: np.ndarray
    velocity_mps: np.ndarray
    command_mps: np.ndarray
    compute_ms: np.ndarray
    effective_samples: np.ndarray


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


def rollout_cost(
    candidates: np.ndarray,
    position: float,
    velocity: float,
    delayed_commands: list[float],
    response_aware: bool,
) -> np.ndarray:
    """Return one scalar cost for each sampled command sequence."""
    sample_positions = np.full(NUM_SAMPLES, position, dtype=np.float64)
    sample_velocities = np.full(NUM_SAMPLES, velocity, dtype=np.float64)
    predicted_positions: list[np.ndarray] = []

    if response_aware:
        # MODEL_DT contains two 20 Hz plant updates.  Carry the already-issued
        # commands into every candidate so the artificial delay is represented.
        queue = [
            np.full(NUM_SAMPLES, value, dtype=np.float64)
            for value in delayed_commands
        ]
        decay = math.exp(-CONTROL_DT / PLANT_TAU)
        substeps_per_prediction = round(MODEL_DT / CONTROL_DT)

        for prediction_step in range(HORIZON_STEPS):
            for _ in range(substeps_per_prediction):
                target_velocity = queue.pop(0)
                queue.append(candidates[:, prediction_step])
                new_velocity = target_velocity + (
                    sample_velocities - target_velocity
                ) * decay
                sample_positions += (
                    0.5 * (sample_velocities + new_velocity) * CONTROL_DT
                )
                sample_velocities = new_velocity
            predicted_positions.append(sample_positions.copy())
    else:
        # This is the idealized assumption being tested: the sampled command is
        # treated as the velocity that is realized immediately.
        for prediction_step in range(HORIZON_STEPS):
            sample_positions += candidates[:, prediction_step] * MODEL_DT
            predicted_positions.append(sample_positions.copy())

    position_matrix = np.stack(predicted_positions, axis=1)
    terminal_error = position_matrix[:, -1] - GOAL_X
    boundary_excess = np.maximum(position_matrix - SAFETY_BOUNDARY_X, 0.0)

    terminal_cost = 120.0 * terminal_error**2
    safety_cost = (
        50000.0 * np.sum(boundary_excess**2, axis=1)
        + 3000.0 * np.any(boundary_excess > 0.0, axis=1)
    )
    smoothness_cost = 2.0 * np.sum(np.diff(candidates, axis=1) ** 2, axis=1)
    effort_cost = np.sum(candidates**2, axis=1)
    return terminal_cost + safety_cost + smoothness_cost + effort_cost


def simulate_controller(
    key: str,
    label: str,
    color: str,
    response_aware: bool,
) -> RunResult:
    rng = np.random.default_rng(7)
    nominal_commands = np.full(HORIZON_STEPS, 0.45, dtype=np.float64)
    delayed_commands = [0.0] * DELAY_STEPS

    position = 0.0
    velocity = 0.0
    position_history: list[float] = []
    velocity_history: list[float] = []
    command_history: list[float] = []
    compute_history: list[float] = []
    effective_sample_history: list[float] = []

    cycle_count = round(SIMULATION_SECONDS / CONTROL_DT)
    plant_decay = math.exp(-CONTROL_DT / PLANT_TAU)

    for _ in range(cycle_count):
        cycle_start = time.perf_counter()
        noise = rng.normal(0.0, 0.25, size=(NUM_SAMPLES, HORIZON_STEPS))
        candidates = np.clip(
            nominal_commands[None, :] + noise,
            -MAX_SPEED,
            MAX_SPEED,
        )
        costs = rollout_cost(
            candidates,
            position,
            velocity,
            delayed_commands,
            response_aware,
        )

        # Same core weighting rule used by MPPI:
        # weight_i = exp(-(cost_i - minimum_cost) / temperature)
        weights = np.exp(-(costs - np.min(costs)) / TEMPERATURE)
        weights /= np.sum(weights)
        nominal_commands = np.sum(weights[:, None] * candidates, axis=0)
        command = float(nominal_commands[0])
        effective_samples = float(1.0 / np.sum(weights**2))
        compute_ms = (time.perf_counter() - cycle_start) * 1000.0

        # Advance the same delayed, first-order plant for one real control cycle.
        delayed_commands.append(command)
        delayed_command = delayed_commands.pop(0)
        new_velocity = delayed_command + (
            velocity - delayed_command
        ) * plant_decay
        position += 0.5 * (velocity + new_velocity) * CONTROL_DT
        velocity = new_velocity

        position_history.append(position)
        velocity_history.append(velocity)
        command_history.append(command)
        compute_history.append(compute_ms)
        effective_sample_history.append(effective_samples)

        # Receding horizon: discard the first element and warm-start the tail.
        nominal_commands = np.concatenate(
            (nominal_commands[1:], nominal_commands[-1:])
        )

    return RunResult(
        key=key,
        label=label,
        color=color,
        time_s=np.arange(cycle_count, dtype=np.float64) * CONTROL_DT,
        position_m=np.asarray(position_history),
        velocity_mps=np.asarray(velocity_history),
        command_mps=np.asarray(command_history),
        compute_ms=np.asarray(compute_history),
        effective_samples=np.asarray(effective_sample_history),
    )


def result_metrics(result: RunResult) -> dict[str, float | int | str]:
    maximum_position = float(np.max(result.position_m))
    safety_margin = SAFETY_BOUNDARY_X - maximum_position
    return {
        "controller": result.key,
        "maximum_position_m": maximum_position,
        "minimum_wall_clearance_m": WALL_X - maximum_position,
        "safety_margin_m": safety_margin,
        "boundary_crossed": int(safety_margin < 0.0),
        "final_position_m": float(result.position_m[-1]),
        "final_goal_error_m": float(abs(result.position_m[-1] - GOAL_X)),
        "peak_velocity_mps": float(np.max(np.abs(result.velocity_mps))),
        "mean_compute_ms": float(np.mean(result.compute_ms)),
        "p95_compute_ms": float(np.percentile(result.compute_ms, 95)),
        "mean_effective_samples": float(np.mean(result.effective_samples)),
    }


def write_results(output_dir: Path, results: list[RunResult]) -> None:
    metrics = [result_metrics(result) for result in results]
    with (output_dir / "closed_loop_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0].keys()))
        writer.writeheader()
        writer.writerows(metrics)

    with (output_dir / "closed_loop_timeseries.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "controller",
                "time_s",
                "position_m",
                "velocity_mps",
                "command_mps",
                "compute_ms",
                "effective_samples",
            ]
        )
        for result in results:
            for row in zip(
                result.time_s,
                result.position_m,
                result.velocity_mps,
                result.command_mps,
                result.compute_ms,
                result.effective_samples,
            ):
                writer.writerow([result.key, *[f"{value:.9f}" for value in row]])


def plot_comparison(output_dir: Path, results: list[RunResult]) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(11.2, 7.4), sharex=True)

    for result in results:
        axes[0].plot(
            result.time_s,
            result.position_m,
            color=result.color,
            linewidth=2.3,
            label=result.label,
        )
        axes[1].plot(
            result.time_s,
            result.command_mps,
            color=result.color,
            linewidth=1.8,
            linestyle="--",
            label=f"{result.label}：指令",
        )
        axes[1].plot(
            result.time_s,
            result.velocity_mps,
            color=result.color,
            linewidth=2.3,
            label=f"{result.label}：实际速度",
        )

    axes[0].axhline(
        GOAL_X, color="#2f855a", linestyle=":", linewidth=1.8, label="目标停止位置"
    )
    axes[0].axhline(
        SAFETY_BOUNDARY_X,
        color="#c53030",
        linestyle="-.",
        linewidth=1.8,
        label="安全边界",
    )
    axes[0].set_ylabel("机器人位置 x (m)")
    axes[0].set_title(
        "最小闭环压力测试：同一延迟/惯性对象，不同MPPI预测模型"
    )
    axes[0].legend(loc="lower right", fontsize=9)

    axes[1].axhline(0.0, color="#555555", linewidth=0.8)
    axes[1].set_ylabel("速度 (m/s)")
    axes[1].set_xlabel("时间 (s)")
    axes[1].legend(loc="upper right", ncol=2, fontsize=8)

    figure.text(
        0.5,
        0.01,
        "说明：延迟0.10s、τ=0.45s均为人为压力测试条件，不代表B2实测值。",
        ha="center",
        fontsize=9,
        color="#555555",
    )
    figure.tight_layout(rect=(0, 0.035, 1, 1))
    figure.savefig(
        output_dir / "closed_loop_comparison.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def create_animation(output_dir: Path, results: list[RunResult]) -> None:
    figure, axis = plt.subplots(figsize=(11.0, 4.8))
    axis.set_xlim(-0.05, 1.08)
    axis.set_ylim(-0.9, 0.9)
    axis.set_xlabel("前进位置 x (m)")
    axis.set_yticks([-0.45, 0.45])
    axis.set_yticklabels(["响应感知模型", "理想模型"])
    axis.axvline(
        SAFETY_BOUNDARY_X,
        color="#c53030",
        linestyle="-.",
        linewidth=2.0,
        label="安全边界",
    )
    axis.axvspan(
        SAFETY_BOUNDARY_X,
        WALL_X,
        color="#fed7d7",
        alpha=0.65,
        label="安全缓冲区",
    )
    axis.axvspan(WALL_X, 1.08, color="#555555", alpha=0.9, label="墙面")
    axis.axvline(
        GOAL_X,
        color="#2f855a",
        linestyle=":",
        linewidth=2.0,
        label="目标停止位置",
    )
    axis.legend(loc="upper left", ncol=4, fontsize=8)
    axis.set_title("8000候选/周期的闭环MPPI压力测试")

    lane_y = {"baseline": 0.45, "aware": -0.45}
    markers = {}
    traces = {}
    status_text = {}
    for result in results:
        y = lane_y[result.key]
        (traces[result.key],) = axis.plot(
            [],
            [],
            color=result.color,
            linewidth=2.2,
            alpha=0.75,
        )
        (markers[result.key],) = axis.plot(
            [],
            [],
            marker="s",
            markersize=15,
            color=result.color,
            markeredgecolor="#222222",
        )
        status_text[result.key] = axis.text(
            0.02,
            y + 0.12,
            "",
            fontsize=9,
            color=result.color,
            weight="bold",
        )

    clock_text = axis.text(
        0.98,
        0.05,
        "",
        transform=axis.transAxes,
        ha="right",
        fontsize=10,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
    )

    frame_indices = np.arange(0, len(results[0].time_s), 2)

    def update(frame_number: int):
        index = int(frame_indices[frame_number])
        artists = [clock_text]
        for result in results:
            y = lane_y[result.key]
            x = float(result.position_m[index])
            trace_y = np.full(index + 1, y)
            traces[result.key].set_data(result.position_m[: index + 1], trace_y)
            markers[result.key].set_data([x], [y])
            crossed = x >= SAFETY_BOUNDARY_X
            state = "越过安全边界" if crossed else "边界内"
            status_text[result.key].set_text(
                f"x={x:.3f}m，v={result.velocity_mps[index]:.3f}m/s，{state}"
            )
            status_text[result.key].set_color("#c53030" if crossed else result.color)
            artists.extend(
                [traces[result.key], markers[result.key], status_text[result.key]]
            )
        clock_text.set_text(f"t={results[0].time_s[index]:.2f}s")
        return artists

    movie = animation.FuncAnimation(
        figure,
        update,
        frames=len(frame_indices),
        interval=100,
        blit=False,
    )
    movie.save(
        output_dir / "closed_loop_demo.gif",
        writer=animation.PillowWriter(fps=10),
        dpi=110,
    )
    plt.close(figure)


def main() -> None:
    configure_plot_style()
    output_dir = Path(__file__).resolve().parent / "results"
    output_dir.mkdir(parents=True, exist_ok=True)

    results = [
        simulate_controller(
            "baseline",
            "理想模型MPPI",
            "#d94841",
            response_aware=False,
        ),
        simulate_controller(
            "aware",
            "响应感知MPPI",
            "#2878b5",
            response_aware=True,
        ),
    ]

    write_results(output_dir, results)
    plot_comparison(output_dir, results)
    create_animation(output_dir, results)

    print(
        "Standalone closed-loop MPPI stress test "
        f"(N={NUM_SAMPLES}, H={HORIZON_STEPS}, dt={MODEL_DT:.2f}s)"
    )
    for result in results:
        metrics = result_metrics(result)
        state = "CROSSED" if metrics["boundary_crossed"] else "SAFE"
        print(
            f"  {result.key:>8}: max_x={metrics['maximum_position_m']:.4f}m, "
            f"safety_margin={metrics['safety_margin_m']:+.4f}m, "
            f"final_error={metrics['final_goal_error_m']:.4f}m, "
            f"{state}"
        )
    print(f"Results written to: {output_dir}")
    print("CAUTION: plant delay/tau are artificial test conditions, not B2 data.")


if __name__ == "__main__":
    main()

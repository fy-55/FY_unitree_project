#!/usr/bin/env python3
"""High-fidelity one-dimensional audit of the current B2 GPU-MPPI chain.

This experiment is not a generic toy MPPI.  It projects the longitudinally
relevant parts of the inspected controller and Nav2 velocity-smoother code onto
one dimension:

* the active YAML parameters are loaded from dog_nav_params.yaml;
* the controller uses the same N=8000, H=5, dt=0.1 s and 20 Hz receding loop;
* warm-start shifting, tail decay, Gaussian/log-normal mixed sampling,
  guidance interpolation, velocity clipping and exponential weighting follow
  the current custom GPU-MPPI implementation;
* the one-dimensional cost is the exact longitudinal projection of the current
  obstacle/progress terms.  No demonstration-only smoothness, effort, safety
  boundary, or terminal quadratic penalty is added;
* the external OPEN_LOOP Nav2 velocity smoother is reproduced, including its
  acceleration/deceleration choice, internal pre-deadband state, and published
  deadband;
* the plant has explicit command delay, first-order response and scale;
* ideal, smoother-aware and full-response-aware rollout models control the same
  execution plant;
* candidate-ranking disagreement is measured on the same 8000 candidates;
* repeated random seeds and a delay/tau sensitivity grid prevent conclusions
  from depending on one lucky sample.

The response parameters remain artificial test conditions until B2
command/odometry data are collected.  This is an offline one-dimensional code
audit, not Nav2/Gazebo or real-robot evidence.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
PARAMS_PATH = ROOT / "src/b2_navigation/params/dog_nav_params.yaml"
CONTROLLER_PATH = ROOT / "src/nav2_custom_plugins/src/mppi_gpu_controller.cpp"
KERNEL_PATH = ROOT / "src/nav2_custom_plugins/src/mppi_gpu_kernels.cu"
REWARDS_PATH = ROOT / "src/nav2_custom_plugins/src/mppi_gpu_rewards.cuh"


@dataclass(frozen=True)
class ControllerParams:
    control_frequency: float
    num_samples: int
    horizon: int
    model_dt: float
    min_v: float
    max_v: float
    action_std_v: float
    temperature: float
    nln_ratio: float
    nln_sigma_mult: float
    noise_decay_rate: float
    noise_scale_floor_vx: float
    lateral_guidance_scale: float
    pure_rotation_ratio: float
    pure_rotation_steps: int
    exploration_decay_start: float
    exploration_decay_end: float
    exploration_decay_floor: float
    spatial_decay_weight: float
    cost_scale: float
    obstacle_ratio: float
    tracking_ratio: float
    speed_ratio: float
    footprint_front: float
    footprint_back: float
    footprint_sample_spacing: float
    lookahead_distance: float
    goal_tolerance: float
    tail_decay: float = 0.5

    @property
    def control_dt(self) -> float:
        return 1.0 / self.control_frequency

    @classmethod
    def load(cls, path: Path = PARAMS_PATH) -> "ControllerParams":
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        controller_server = data["controller_server"]["ros__parameters"]
        follow = controller_server["FollowPath"]
        goal_checker = controller_server["general_goal_checker"]
        return cls(
            control_frequency=float(controller_server["controller_frequency"]),
            num_samples=int(follow["num_samples"]),
            horizon=int(follow["prediction_horizon"]),
            model_dt=float(follow["dt"]),
            min_v=float(follow["min_v"]),
            max_v=float(follow["max_v"]),
            action_std_v=float(follow["action_std_v"]),
            temperature=float(follow["lambda"]),
            nln_ratio=float(follow["nln_ratio"]),
            # This parameter is absent from the active YAML and therefore uses
            # the C++ member default in mppi_gpu_controller.hpp.
            nln_sigma_mult=float(follow.get("nln_sigma_mult", 3.0)),
            noise_decay_rate=float(follow["noise_decay_rate"]),
            noise_scale_floor_vx=float(follow["noise_scale_floor_vx"]),
            lateral_guidance_scale=float(follow["lateral_guidance_scale"]),
            pure_rotation_ratio=float(follow["pure_rotation_ratio"]),
            pure_rotation_steps=int(follow["pure_rotation_steps"]),
            exploration_decay_start=float(
                follow.get("exploration_decay_start", 3.0)
            ),
            exploration_decay_end=float(
                follow.get("exploration_decay_end", 0.5)
            ),
            exploration_decay_floor=float(
                follow.get("exploration_decay_floor", 0.3)
            ),
            spatial_decay_weight=float(follow.get("spatial_decay_weight", 0.5)),
            cost_scale=float(follow["cost_scale"]),
            obstacle_ratio=float(follow["obstacle_ratio"]),
            tracking_ratio=float(follow["tracking_ratio"]),
            speed_ratio=float(follow["speed_ratio"]),
            footprint_front=float(follow["footprint_front"]),
            footprint_back=float(follow["footprint_back"]),
            footprint_sample_spacing=float(follow["footprint_sample_spacing"]),
            lookahead_distance=float(follow["min_lookahead_dist"]),
            goal_tolerance=float(goal_checker["xy_goal_tolerance"]),
        )


@dataclass(frozen=True)
class SmootherParams:
    frequency: float
    min_velocity: float
    max_velocity: float
    max_accel: float
    max_decel: float
    deadband: float
    feedback: str
    scale_velocities: bool

    @classmethod
    def load(cls, path: Path = PARAMS_PATH) -> "SmootherParams":
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        params = data["velocity_smoother"]["ros__parameters"]
        return cls(
            frequency=float(params["smoothing_frequency"]),
            min_velocity=float(params["min_velocity"][0]),
            max_velocity=float(params["max_velocity"][0]),
            max_accel=float(params["max_accel"][0]),
            max_decel=float(params["max_decel"][0]),
            deadband=float(params["deadband_velocity"][0]),
            feedback=str(params["feedback"]),
            scale_velocities=bool(params["scale_velocities"]),
        )


@dataclass(frozen=True)
class PlantParams:
    key: str
    delay_s: float
    tau_s: float
    scale: float


@dataclass(frozen=True)
class Task:
    key: str
    label: str
    goal_x: float
    duration_s: float
    wall_x: float | None
    desired_clearance: float


@dataclass
class SmootherState:
    internal_velocity: float = 0.0


@dataclass
class PlantState:
    position: float
    velocity: float
    queue: deque[float]


@dataclass
class ControllerState:
    sequence: np.ndarray | None = None


MODEL_LABELS = {
    "ideal": "理想速度模型",
    "smoother": "速度平滑感知模型",
    "response": "完整响应感知模型",
}
MODEL_COLORS = {
    "ideal": "#d94841",
    "smoother": "#e6a700",
    "response": "#2878b5",
}

PLANT_SCENARIOS = (
    PlantParams("smoother_only", delay_s=0.00, tau_s=0.00, scale=1.00),
    PlantParams("mild", delay_s=0.05, tau_s=0.15, scale=0.90),
    PlantParams("stress", delay_s=0.10, tau_s=0.45, scale=1.00),
)

TASKS = (
    Task(
        key="goal_stop",
        label="无墙目标停车",
        goal_x=1.00,
        duration_s=6.0,
        wall_x=None,
        desired_clearance=0.0,
    ),
    Task(
        key="wall_stop",
        label="合法墙前目标停车",
        # The robot center target leaves 0.15 m physical front clearance.
        # The active 0.35 m Nav2 goal tolerance is intentionally retained.
        goal_x=0.85,
        duration_s=6.0,
        wall_x=1.50,
        desired_clearance=0.10,
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


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def smoother_update(
    target: np.ndarray | float,
    current_internal: np.ndarray | float,
    params: SmootherParams,
) -> tuple[np.ndarray | float, np.ndarray | float]:
    """Reproduce Humble nav2_velocity_smoother for the longitudinal axis.

    OPEN_LOOP uses the previous pre-deadband command as current velocity.
    The deadband is applied only to the published value, after last_cmd_ is
    stored in the original implementation.
    """
    target_array = np.clip(target, params.min_velocity, params.max_velocity)
    current_array = np.asarray(current_internal)
    accelerating = (
        (np.abs(target_array) >= np.abs(current_array))
        & (current_array * target_array >= 0.0)
    )
    upper = np.where(
        accelerating,
        params.max_accel / params.frequency,
        -params.max_decel / params.frequency,
    )
    lower = -upper
    internal = current_array + np.clip(
        target_array - current_array, lower, upper
    )
    published = np.where(np.abs(internal) < params.deadband, 0.0, internal)
    if np.ndim(internal) == 0:
        return float(internal), float(published)
    return internal, published


def delay_steps(plant: PlantParams, control_dt: float) -> int:
    steps = int(round(plant.delay_s / control_dt))
    if not math.isclose(steps * control_dt, plant.delay_s, abs_tol=1e-9):
        raise ValueError("Plant delay must be an integer multiple of control_dt")
    return steps


def plant_update(
    published_command: float,
    state: PlantState,
    plant: PlantParams,
    control_dt: float,
) -> None:
    if state.queue:
        delayed = state.queue.popleft()
        state.queue.append(published_command)
    else:
        delayed = published_command
    target_velocity = plant.scale * delayed
    old_velocity = state.velocity
    if plant.tau_s <= 0.0:
        new_velocity = target_velocity
        displacement = new_velocity * control_dt
    else:
        decay = math.exp(-control_dt / plant.tau_s)
        new_velocity = target_velocity + (
            old_velocity - target_velocity
        ) * decay
        displacement = 0.5 * (old_velocity + new_velocity) * control_dt
    state.position += displacement
    state.velocity = new_velocity


def inflation_cost_at_points(
    points: np.ndarray,
    wall_x: float,
    inflation_radius: float = 0.50,
    inscribed_radius: float = 0.30,
    cost_scaling_factor: float = 10.0,
) -> np.ndarray:
    """Plane-wall equivalent of Nav2's inflation cost (0..254)."""
    distance = wall_x - points
    values = np.zeros_like(points, dtype=np.float64)
    values[distance <= 0.0] = 254.0
    inscribed = (distance > 0.0) & (distance <= inscribed_radius)
    values[inscribed] = 253.0
    inflated = (distance > inscribed_radius) & (distance <= inflation_radius)
    values[inflated] = 252.0 * np.exp(
        -cost_scaling_factor * (distance[inflated] - inscribed_radius)
    )
    return values


def obstacle_cost(
    center_positions: np.ndarray,
    task: Task,
    controller: ControllerParams,
) -> np.ndarray:
    """Longitudinal projection of compute_obstacle_cost().

    The CUDA kernel caps footprint samples at 10 along each dimension.  For a
    plane wall, all lateral samples have the same value and cancel in the mean,
    so only the ten longitudinal samples are required.
    """
    if task.wall_x is None:
        return np.zeros_like(center_positions)
    length = controller.footprint_front + controller.footprint_back
    count = int(math.ceil(length / controller.footprint_sample_spacing)) + 1
    count = max(2, min(10, count))
    offsets = np.linspace(
        -controller.footprint_back, controller.footprint_front, count
    )
    sample_points = center_positions[..., None] + offsets
    cell_costs = inflation_cost_at_points(sample_points, task.wall_x)
    normalized = cell_costs / 255.0
    return np.mean(normalized**4, axis=-1)


def exploration_scale(distance: float, params: ControllerParams) -> float:
    result = 1.0
    if distance < params.exploration_decay_start:
        ratio = (
            distance - params.exploration_decay_end
        ) / (
            params.exploration_decay_start - params.exploration_decay_end
        )
        ratio = float(np.clip(ratio, 0.0, 1.0))
        result = params.exploration_decay_floor + (
            1.0 - params.exploration_decay_floor
        ) * (1.0 - ratio)
    return result


def mixed_noise(
    rng: np.random.Generator, shape: tuple[int, int], params: ControllerParams
) -> np.ndarray:
    gaussian = rng.normal(0.0, params.action_std_v, size=shape)
    use_lognormal = rng.random(shape) < params.nln_ratio
    magnitude = (
        rng.lognormal(
            mean=0.0,
            sigma=params.action_std_v * params.nln_sigma_mult,
            size=shape,
        )
        - 1.0
    )
    signs = np.where(rng.random(shape) < 0.5, -1.0, 1.0)
    return np.where(use_lognormal, signs * magnitude, gaussian)


def generate_candidates(
    base: np.ndarray,
    position: float,
    task: Task,
    params: ControllerParams,
    rng: np.random.Generator,
) -> np.ndarray:
    noise = mixed_noise(rng, (params.num_samples, params.horizon), params)
    target_direction = 1.0 if task.goal_x >= position else -1.0
    lookahead_target = (
        min(position + params.lookahead_distance, task.goal_x)
        if target_direction > 0.0
        else task.goal_x
    )
    distance = abs(lookahead_target - position)
    explore = exploration_scale(distance, params)
    blockage = 0.0
    if task.wall_x is not None:
        current_cost = float(
            inflation_cost_at_points(
                np.asarray([position]), task.wall_x
            )[0]
        )
        lookahead_cost = float(
            inflation_cost_at_points(
                np.asarray([lookahead_target]), task.wall_x
            )[0]
        )
        difference = lookahead_cost - current_cost
        if difference > 60.0:
            blockage = 1.0
        elif difference > 20.0:
            blockage = (difference - 20.0) / 40.0
    dynamic_explore = max(explore, 0.3 + blockage * 0.7)
    dynamic_noise_floor = params.noise_scale_floor_vx + blockage * 0.4

    candidates = np.empty_like(noise)
    cumulative_distance = np.zeros(params.num_samples)
    maximum_travel = params.max_v * params.model_dt * params.horizon
    for step in range(params.horizon):
        temporal_progress = step / max(1, params.horizon - 1)
        spatial_progress = np.minimum(
            1.0, cumulative_distance / max(0.01, maximum_travel)
        )
        effective_progress = (
            (1.0 - params.spatial_decay_weight) * temporal_progress
            + params.spatial_decay_weight * spatial_progress
        )
        noise_scale = np.maximum(
            dynamic_noise_floor,
            (1.0 - params.noise_decay_rate * effective_progress)
            * dynamic_explore,
        )
        reference_speed = max(abs(base[step]), params.max_v * 0.3)
        candidates[:, step] = (
            (1.0 - params.lateral_guidance_scale)
            * (base[step] + noise[:, step] * noise_scale)
            + params.lateral_guidance_scale
            * target_direction
            * reference_speed
        )
        candidates[:, step] = np.clip(
            candidates[:, step], params.min_v, params.max_v
        )
        cumulative_distance += (
            np.abs(candidates[:, step]) * params.model_dt
        )

    rotation_count = int(params.pure_rotation_ratio * params.num_samples)
    if rotation_count > 0:
        candidates[:rotation_count, : params.pure_rotation_steps] = 0.0
    return candidates


def rollout_candidates(
    candidates: np.ndarray,
    model: str,
    initial_position: float,
    initial_velocity: float,
    smoother_internal: float,
    queued_commands: Iterable[float],
    task: Task,
    controller: ControllerParams,
    smoother: SmootherParams,
    plant: PlantParams,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return costs, predicted positions and realized rollout velocities."""
    sample_count, horizon = candidates.shape
    positions = np.full(sample_count, initial_position)
    velocities = np.full(sample_count, initial_velocity)
    position_history = np.empty((sample_count, horizon))
    velocity_history = np.empty((sample_count, horizon))

    if model == "ideal":
        for step in range(horizon):
            velocities = candidates[:, step]
            positions += velocities * controller.model_dt
            position_history[:, step] = positions
            velocity_history[:, step] = velocities
    else:
        substeps = int(round(controller.model_dt / controller.control_dt))
        if not math.isclose(
            substeps * controller.control_dt,
            controller.model_dt,
            abs_tol=1e-9,
        ):
            raise ValueError("model_dt must be a multiple of control_dt")
        smoother_states = np.full(sample_count, smoother_internal)
        queue_values = list(queued_commands)
        queue = (
            np.tile(np.asarray(queue_values), (sample_count, 1))
            if queue_values
            else np.empty((sample_count, 0))
        )
        decay = (
            math.exp(-controller.control_dt / plant.tau_s)
            if plant.tau_s > 0.0
            else 0.0
        )
        for step in range(horizon):
            for _ in range(substeps):
                smoother_states, published = smoother_update(
                    candidates[:, step], smoother_states, smoother
                )
                if model == "smoother":
                    velocities = np.asarray(published)
                    positions += velocities * controller.control_dt
                elif model == "response":
                    if queue.shape[1]:
                        delayed = queue[:, 0].copy()
                        queue[:, :-1] = queue[:, 1:]
                        queue[:, -1] = published
                    else:
                        delayed = np.asarray(published)
                    target = plant.scale * delayed
                    old_velocities = velocities.copy()
                    if plant.tau_s <= 0.0:
                        velocities = target
                        positions += velocities * controller.control_dt
                    else:
                        velocities = target + (velocities - target) * decay
                        positions += (
                            0.5
                            * (old_velocities + velocities)
                            * controller.control_dt
                        )
                else:
                    raise ValueError(f"Unknown rollout model: {model}")
            position_history[:, step] = positions
            velocity_history[:, step] = velocities

    obstacle_acc = obstacle_cost(position_history, task, controller).sum(axis=1)
    target_direction = 1.0 if task.goal_x >= initial_position else -1.0
    aligned = velocity_history * target_direction
    # Exact 1-D branches of compute_speed_reward(): aligned motion receives
    # -speed; opposite motion has pi angular error and receives 9.712...*speed.
    reverse_multiplier = 5.0 + 3.0 * math.pi / 2.0
    progress_terms = np.where(
        aligned >= 0.0,
        -np.abs(velocity_history),
        reverse_multiplier * np.abs(velocity_history),
    )
    progress_acc = progress_terms.sum(axis=1)
    progress_acc += np.abs(position_history[:, -1] - task.goal_x)

    inverse_horizon = 1.0 / controller.horizon
    # On a straight 1-D path, PathAlign and PathAngle are exactly zero.
    costs = controller.cost_scale * (
        controller.obstacle_ratio * obstacle_acc * inverse_horizon
        + controller.tracking_ratio * 0.0
        + controller.speed_ratio * progress_acc * inverse_horizon
    )
    return costs, position_history, velocity_history


def normalized_weights(costs: np.ndarray, temperature: float) -> np.ndarray:
    weights = np.exp(-(costs - np.min(costs)) / temperature)
    total = float(np.sum(weights))
    if not math.isfinite(total) or total <= 0.0:
        raise FloatingPointError("MPPI weights are not finite")
    return weights / total


def rank_metrics(
    assumed_costs: np.ndarray,
    response_costs: np.ndarray,
    assumed_weights: np.ndarray,
    response_weights: np.ndarray,
    candidates: np.ndarray,
) -> dict[str, float]:
    count = len(assumed_costs)
    assumed_order = np.argsort(assumed_costs)
    response_order = np.argsort(response_costs)
    assumed_ranks = np.empty(count, dtype=np.float64)
    response_ranks = np.empty(count, dtype=np.float64)
    assumed_ranks[assumed_order] = np.arange(count)
    response_ranks[response_order] = np.arange(count)
    spearman = float(np.corrcoef(assumed_ranks, response_ranks)[0, 1])
    top_count = max(1, int(round(count * 0.01)))
    top_overlap = len(
        set(assumed_order[:top_count]).intersection(response_order[:top_count])
    ) / top_count
    assumed_best = int(assumed_order[0])
    assumed_best_response_percentile = (
        100.0 * response_ranks[assumed_best] / max(1, count - 1)
    )
    assumed_command = float(np.sum(assumed_weights * candidates[:, 0]))
    response_command = float(np.sum(response_weights * candidates[:, 0]))
    return {
        "rank_spearman": spearman,
        "top_1pct_overlap": top_overlap,
        "ideal_best_response_percentile": assumed_best_response_percentile,
        "weighted_command_difference_mps": response_command - assumed_command,
    }


def predict_selected_next(
    raw_command: float,
    model: str,
    plant_state: PlantState,
    smoother_state: SmootherState,
    controller: ControllerParams,
    smoother: SmootherParams,
    plant: PlantParams,
) -> float:
    if model == "ideal":
        return plant_state.position + raw_command * controller.control_dt
    internal, published = smoother_update(
        raw_command, smoother_state.internal_velocity, smoother
    )
    del internal
    if model == "smoother":
        return plant_state.position + published * controller.control_dt
    queue = deque(plant_state.queue)
    clone = PlantState(
        position=plant_state.position,
        velocity=plant_state.velocity,
        queue=queue,
    )
    plant_update(published, clone, plant, controller.control_dt)
    return clone.position


def controller_cycle(
    model: str,
    state: ControllerState,
    plant_state: PlantState,
    smoother_state: SmootherState,
    task: Task,
    controller: ControllerParams,
    smoother: SmootherParams,
    plant: PlantParams,
    rng: np.random.Generator,
    audit_ranking: bool,
) -> tuple[float, dict[str, float]]:
    start = time.perf_counter()
    if state.sequence is None:
        target_direction = 1.0 if task.goal_x >= plant_state.position else -1.0
        lookahead_target = (
            min(
                plant_state.position + controller.lookahead_distance,
                task.goal_x,
            )
            if target_direction > 0.0
            else task.goal_x
        )
        distance = abs(lookahead_target - plant_state.position)
        initial_speed = min(controller.max_v, distance / controller.model_dt)
        base = np.full(
            controller.horizon, target_direction * initial_speed
        )
    else:
        base = np.empty_like(state.sequence)
        base[:-1] = state.sequence[1:]
        base[-1] = state.sequence[-1] * controller.tail_decay

    candidates = generate_candidates(
        base, plant_state.position, task, controller, rng
    )
    costs, _, _ = rollout_candidates(
        candidates,
        model,
        plant_state.position,
        plant_state.velocity,
        smoother_state.internal_velocity,
        plant_state.queue,
        task,
        controller,
        smoother,
        plant,
    )
    weights = normalized_weights(costs, controller.temperature)
    optimal_sequence = np.sum(weights[:, None] * candidates, axis=0)
    state.sequence = optimal_sequence
    raw_command = float(
        np.clip(optimal_sequence[0], controller.min_v, controller.max_v)
    )
    predicted_next = predict_selected_next(
        raw_command,
        model,
        plant_state,
        smoother_state,
        controller,
        smoother,
        plant,
    )
    data = {
        "predicted_next_position_m": predicted_next,
        "effective_samples": float(1.0 / np.sum(weights**2)),
        "minimum_assumed_cost": float(np.min(costs)),
        "rank_spearman": math.nan,
        "top_1pct_overlap": math.nan,
        "ideal_best_response_percentile": math.nan,
        "weighted_command_difference_mps": math.nan,
    }

    if audit_ranking:
        response_costs, _, _ = rollout_candidates(
            candidates,
            "response",
            plant_state.position,
            plant_state.velocity,
            smoother_state.internal_velocity,
            plant_state.queue,
            task,
            controller,
            smoother,
            plant,
        )
        response_weights = normalized_weights(
            response_costs, controller.temperature
        )
        data.update(
            rank_metrics(
                costs,
                response_costs,
                weights,
                response_weights,
                candidates,
            )
        )

    data["compute_ms"] = (time.perf_counter() - start) * 1000.0
    return raw_command, data


def settling_time(
    times: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    goal: float,
    position_tolerance: float = 0.05,
    velocity_tolerance: float = 0.05,
) -> float:
    settled = (np.abs(positions - goal) <= position_tolerance) & (
        np.abs(velocities) <= velocity_tolerance
    )
    suffix_all = np.logical_and.accumulate(settled[::-1])[::-1]
    indices = np.flatnonzero(suffix_all)
    return float(times[indices[0]]) if len(indices) else math.nan


def simulate_run(
    model: str,
    task: Task,
    plant: PlantParams,
    seed: int,
    controller: ControllerParams,
    smoother: SmootherParams,
    audit_ranking: bool,
) -> tuple[dict[str, float | int | str], list[dict[str, float | int | str]]]:
    rng = np.random.default_rng(seed)
    controller_state = ControllerState()
    smoother_state = SmootherState()
    plant_state = PlantState(
        position=0.0,
        velocity=0.0,
        queue=deque(
            [0.0] * delay_steps(plant, controller.control_dt)
        ),
    )
    rows: list[dict[str, float | int | str]] = []
    cycle_count = int(round(task.duration_s / controller.control_dt))
    goal_reached = False
    goal_reached_time = math.nan

    for cycle in range(cycle_count):
        if (
            not goal_reached
            and abs(plant_state.position - task.goal_x)
            <= controller.goal_tolerance
        ):
            # Nav2's goal checker ends FollowPath and the command chain receives
            # zero.  Completion is latched even if residual motion later leaves
            # the tolerance region.
            goal_reached = True
            goal_reached_time = cycle * controller.control_dt
        if goal_reached:
            raw_command = 0.0
            cycle_data = {
                "predicted_next_position_m": predict_selected_next(
                    raw_command,
                    model,
                    plant_state,
                    smoother_state,
                    controller,
                    smoother,
                    plant,
                ),
                "effective_samples": math.nan,
                "minimum_assumed_cost": math.nan,
                "rank_spearman": math.nan,
                "top_1pct_overlap": math.nan,
                "ideal_best_response_percentile": math.nan,
                "weighted_command_difference_mps": math.nan,
                "compute_ms": math.nan,
            }
        else:
            raw_command, cycle_data = controller_cycle(
                model,
                controller_state,
                plant_state,
                smoother_state,
                task,
                controller,
                smoother,
                plant,
                rng,
                audit_ranking,
            )
        internal, published = smoother_update(
            raw_command, smoother_state.internal_velocity, smoother
        )
        smoother_state.internal_velocity = float(internal)
        position_before = plant_state.position
        plant_update(published, plant_state, plant, controller.control_dt)
        rows.append(
            {
                "task": task.key,
                "plant": plant.key,
                "model": model,
                "seed": seed,
                "cycle": cycle,
                "time_s": (cycle + 1) * controller.control_dt,
                "goal_reached": int(goal_reached),
                "position_m": plant_state.position,
                "velocity_mps": plant_state.velocity,
                "raw_mppi_command_mps": raw_command,
                "smoother_internal_mps": smoother_state.internal_velocity,
                "smoothed_published_mps": published,
                "actual_displacement_m": plant_state.position - position_before,
                "first_cycle_prediction_error_m": (
                    plant_state.position
                    - cycle_data["predicted_next_position_m"]
                ),
                **cycle_data,
            }
        )

    times = np.asarray([row["time_s"] for row in rows], dtype=float)
    positions = np.asarray([row["position_m"] for row in rows], dtype=float)
    velocities = np.asarray([row["velocity_mps"] for row in rows], dtype=float)
    raw = np.asarray([row["raw_mppi_command_mps"] for row in rows], dtype=float)
    smoothed = np.asarray(
        [row["smoothed_published_mps"] for row in rows], dtype=float
    )
    first_errors = np.asarray(
        [row["first_cycle_prediction_error_m"] for row in rows], dtype=float
    )
    compute = np.asarray([row["compute_ms"] for row in rows], dtype=float)
    ranking = np.asarray([row["rank_spearman"] for row in rows], dtype=float)
    overlap = np.asarray([row["top_1pct_overlap"] for row in rows], dtype=float)

    maximum_position = float(np.max(positions))
    if task.wall_x is None:
        minimum_clearance = math.nan
        safety_margin = math.nan
        collision = 0
        safety_violation = 0
    else:
        minimum_clearance = (
            task.wall_x - (maximum_position + controller.footprint_front)
        )
        safety_margin = minimum_clearance - task.desired_clearance
        collision = int(minimum_clearance < 0.0)
        safety_violation = int(safety_margin < 0.0)

    summary: dict[str, float | int | str] = {
        "task": task.key,
        "plant": plant.key,
        "model": model,
        "seed": seed,
        "delay_s": plant.delay_s,
        "tau_s": plant.tau_s,
        "scale": plant.scale,
        "maximum_position_m": maximum_position,
        "maximum_goal_overshoot_m": max(
            0.0, maximum_position - task.goal_x
        ),
        "final_position_m": float(positions[-1]),
        "final_goal_error_m": float(abs(positions[-1] - task.goal_x)),
        "goal_reached": int(goal_reached),
        "goal_reached_time_s": goal_reached_time,
        "settling_time_s": settling_time(
            times, positions, velocities, task.goal_x
        ),
        "minimum_wall_clearance_m": minimum_clearance,
        "safety_margin_m": safety_margin,
        "collision": collision,
        "safety_violation": safety_violation,
        "peak_actual_speed_mps": float(np.max(np.abs(velocities))),
        "mean_abs_raw_to_smoothed_mps": float(
            np.mean(np.abs(raw - smoothed))
        ),
        "mean_abs_smoothed_to_actual_mps": float(
            np.mean(np.abs(smoothed - velocities))
        ),
        "mean_abs_first_cycle_prediction_error_m": float(
            np.mean(np.abs(first_errors))
        ),
        "p95_abs_first_cycle_prediction_error_m": float(
            np.percentile(np.abs(first_errors), 95)
        ),
        "raw_command_total_variation_mps": float(np.sum(np.abs(np.diff(raw)))),
        "mean_compute_ms": float(np.nanmean(compute)),
        "p95_compute_ms": float(np.nanpercentile(compute, 95)),
        "mean_rank_spearman": (
            float(np.nanmean(ranking))
            if np.any(np.isfinite(ranking))
            else math.nan
        ),
        "mean_top_1pct_overlap": (
            float(np.nanmean(overlap))
            if np.any(np.isfinite(overlap))
            else math.nan
        ),
    }
    return summary, rows


def mean_ci95(values: list[float]) -> tuple[float, float]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan, math.nan
    mean = statistics.fmean(finite)
    if len(finite) < 2:
        return mean, math.nan
    return mean, 1.96 * statistics.stdev(finite) / math.sqrt(len(finite))


def aggregate_runs(runs: list[dict[str, object]]) -> list[dict[str, object]]:
    keys = sorted(
        {
            (str(row["task"]), str(row["plant"]), str(row["model"]))
            for row in runs
        }
    )
    output: list[dict[str, object]] = []
    metrics = (
        "maximum_goal_overshoot_m",
        "final_goal_error_m",
        "settling_time_s",
        "goal_reached_time_s",
        "minimum_wall_clearance_m",
        "safety_margin_m",
        "mean_abs_first_cycle_prediction_error_m",
        "p95_abs_first_cycle_prediction_error_m",
        "mean_rank_spearman",
        "mean_top_1pct_overlap",
        "mean_compute_ms",
    )
    for task, plant, model in keys:
        group = [
            row
            for row in runs
            if row["task"] == task
            and row["plant"] == plant
            and row["model"] == model
        ]
        aggregate: dict[str, object] = {
            "task": task,
            "plant": plant,
            "model": model,
            "runs": len(group),
            "goal_reached_rate": statistics.fmean(
                float(row["goal_reached"]) for row in group
            ),
            "collision_rate": statistics.fmean(
                float(row["collision"]) for row in group
            ),
            "safety_violation_rate": statistics.fmean(
                float(row["safety_violation"]) for row in group
            ),
        }
        for metric in metrics:
            mean, ci = mean_ci95([float(row[metric]) for row in group])
            aggregate[f"{metric}_mean"] = mean
            aggregate[f"{metric}_ci95"] = ci
        output.append(aggregate)
    return output


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, object]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [dict(row) for row in csv.DictReader(stream)]


def plot_execution_chain(
    output_dir: Path,
    rows: list[dict[str, object]],
    controller: ControllerParams,
) -> None:
    """Bridge raw MPPI commands to smoother output, velocity, and position.

    Commands are drawn at the beginning of each 20 Hz control interval.  Plant
    velocity and position are drawn at the end of that same interval, so the
    lower two panels compare predictions and measurements on one time basis.
    """
    selected = sorted(
        (
            row
            for row in rows
            if row["task"] == "wall_stop"
            and row["plant"] == "stress"
            and row["model"] == "ideal"
            and int(row["seed"]) == 7
        ),
        key=lambda row: int(row["cycle"]),
    )
    if not selected:
        raise ValueError(
            "No wall_stop/stress/ideal/seed=7 rows for execution-chain plot"
        )

    interval_ends = np.asarray(
        [float(row["time_s"]) for row in selected], dtype=float
    )
    interval_starts = interval_ends - controller.control_dt
    raw = np.asarray(
        [float(row["raw_mppi_command_mps"]) for row in selected],
        dtype=float,
    )
    smoothed = np.asarray(
        [float(row["smoothed_published_mps"]) for row in selected],
        dtype=float,
    )
    actual_velocity = np.asarray(
        [float(row["velocity_mps"]) for row in selected], dtype=float
    )
    actual_position = np.asarray(
        [float(row["position_m"]) for row in selected], dtype=float
    )
    predicted_position = np.asarray(
        [float(row["predicted_next_position_m"]) for row in selected],
        dtype=float,
    )
    actual_displacement = np.asarray(
        [float(row["actual_displacement_m"]) for row in selected],
        dtype=float,
    )
    position_before = np.concatenate(([0.0], actual_position[:-1]))
    predicted_displacement = predicted_position - position_before

    # Step commands are extended to the final interval boundary.  Actual
    # velocity includes the known zero initial state at t=0.
    command_times = np.append(interval_starts, interval_ends[-1])
    raw_steps = np.append(raw, raw[-1])
    smoothed_steps = np.append(smoothed, smoothed[-1])
    velocity_times = np.concatenate(([0.0], interval_ends))
    velocity_values = np.concatenate(([0.0], actual_velocity))

    task = next(item for item in TASKS if item.key == "wall_stop")
    contact_center = task.wall_x - controller.footprint_front
    safety_center = contact_center - task.desired_clearance
    goal_indices = [
        index
        for index, row in enumerate(selected)
        if int(row["goal_reached"]) == 1
    ]
    goal_latch_time = (
        interval_starts[goal_indices[0]] if goal_indices else math.nan
    )

    figure, axes = plt.subplots(3, 1, figsize=(12.0, 10.2), sharex=True)
    axes[0].step(
        command_times,
        raw_steps,
        where="post",
        color=MODEL_COLORS["ideal"],
        linewidth=1.8,
        linestyle="--",
        label="MPPI原始指令 $u_{raw}$（区间起点发出）",
    )
    axes[0].step(
        command_times,
        smoothed_steps,
        where="post",
        color=MODEL_COLORS["smoother"],
        linewidth=2.1,
        label="外部速度平滑器输出 $u_{smooth}$",
    )
    axes[0].plot(
        velocity_times,
        velocity_values,
        color=MODEL_COLORS["response"],
        linewidth=2.3,
        marker="o",
        markersize=2.6,
        markevery=4,
        label="人工对象实际速度 $v_{actual}$（区间终点）",
    )
    axes[0].set_ylabel("速度 (m/s)")
    axes[0].set_title(
        "执行链桥梁图：原始指令 → 外部平滑器 → 实际速度 → 位置\n"
        "wall_stop / stress / ideal rollout / seed=7（人工条件，不是B2实测）"
    )
    axes[0].legend(fontsize=9, ncol=2)

    axes[1].plot(
        interval_ends,
        1000.0 * predicted_displacement,
        color=MODEL_COLORS["ideal"],
        linewidth=1.9,
        linestyle="--",
        label="理想模型预测的本周期位移",
    )
    axes[1].plot(
        interval_ends,
        1000.0 * actual_displacement,
        color=MODEL_COLORS["response"],
        linewidth=2.2,
        label="经过平滑器和人工对象后的实际位移",
    )
    axes[1].fill_between(
        interval_ends,
        1000.0 * predicted_displacement,
        1000.0 * actual_displacement,
        color="#805ad5",
        alpha=0.13,
        label="同一0.05秒周期的预测差",
    )
    axes[1].set_ylabel("单周期位移 (mm)")
    axes[1].legend(fontsize=9, ncol=2)

    axes[2].plot(
        interval_ends,
        predicted_position,
        color=MODEL_COLORS["ideal"],
        linewidth=1.8,
        linestyle="--",
        label="理想模型预测的下一周期位置",
    )
    axes[2].plot(
        interval_ends,
        actual_position,
        color=MODEL_COLORS["response"],
        linewidth=2.3,
        label="下一周期重新读到的实际位置",
    )
    axes[2].axhline(
        task.goal_x,
        color="#2f855a",
        linestyle=":",
        linewidth=1.5,
        label="目标中心位置",
    )
    axes[2].axhline(
        safety_center,
        color="#c53030",
        linestyle="-.",
        linewidth=1.4,
        label="安全中心边界",
    )
    axes[2].axhline(
        contact_center,
        color="#333333",
        linestyle="--",
        linewidth=1.3,
        label="物理接触边界",
    )
    axes[2].set_ylabel("机器人中心位置 x (m)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].legend(fontsize=8.5, ncol=2)

    if math.isfinite(goal_latch_time):
        for axis in axes:
            axis.axvline(
                goal_latch_time,
                color="#4a5568",
                linestyle=":",
                linewidth=1.2,
                alpha=0.85,
            )
        axes[0].annotate(
            "GoalChecker锁定\n原始指令变为0",
            xy=(goal_latch_time, 0.0),
            xytext=(goal_latch_time + 0.25, 0.22),
            arrowprops={"arrowstyle": "->", "color": "#4a5568"},
            fontsize=9,
            color="#2d3748",
        )

    figure.tight_layout()
    figure.savefig(
        output_dir / "execution_chain_bridge.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def plot_representative(
    output_dir: Path,
    rows: list[dict[str, object]],
    controller: ControllerParams,
) -> None:
    selected = [
        row
        for row in rows
        if row["task"] == "wall_stop"
        and row["plant"] == "stress"
        and int(row["seed"]) == 7
    ]
    figure, axes = plt.subplots(3, 1, figsize=(11.5, 9.0), sharex=True)
    task = next(item for item in TASKS if item.key == "wall_stop")
    for model in MODEL_LABELS:
        group = [row for row in selected if row["model"] == model]
        times = [float(row["time_s"]) for row in group]
        axes[0].plot(
            times,
            [float(row["position_m"]) for row in group],
            color=MODEL_COLORS[model],
            linewidth=2.1,
            label=MODEL_LABELS[model],
        )
        axes[1].plot(
            times,
            [float(row["raw_mppi_command_mps"]) for row in group],
            color=MODEL_COLORS[model],
            linewidth=1.5,
            linestyle="--",
            label=f"{MODEL_LABELS[model]}：MPPI输出",
        )
        axes[1].plot(
            times,
            [float(row["smoothed_published_mps"]) for row in group],
            color=MODEL_COLORS[model],
            linewidth=2.0,
            label=f"{MODEL_LABELS[model]}：平滑后",
        )
        axes[2].plot(
            times,
            [
                1000.0 * abs(float(row["first_cycle_prediction_error_m"]))
                for row in group
            ],
            color=MODEL_COLORS[model],
            linewidth=2.0,
            label=MODEL_LABELS[model],
        )
    axes[0].axhline(
        task.goal_x, color="#2f855a", linestyle=":", label="目标中心位置"
    )
    contact_center = task.wall_x - controller.footprint_front
    safety_center = contact_center - task.desired_clearance
    axes[0].axhline(
        safety_center, color="#c53030", linestyle="-.", label="安全中心边界"
    )
    axes[0].axhline(
        contact_center, color="#333333", linestyle="--", label="物理接触边界"
    )
    axes[0].set_ylabel("机器人中心位置 x (m)")
    axes[0].set_title(
        "完整一维闭环：相同速度平滑器和人工响应对象（stress，seed=7）"
    )
    axes[0].legend(fontsize=8, ncol=2)
    axes[1].set_ylabel("速度 (m/s)")
    axes[1].legend(fontsize=8, ncol=2)
    axes[2].set_ylabel("|下一周期位置预测误差| (mm)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(
        output_dir / "representative_closed_loop.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def plot_ranking(output_dir: Path, rows: list[dict[str, object]]) -> None:
    selected = [
        row
        for row in rows
        if row["task"] == "wall_stop"
        and row["plant"] == "stress"
        and row["model"] == "ideal"
        and int(row["seed"]) == 7
    ]
    times = [float(row["time_s"]) for row in selected]
    figure, axes = plt.subplots(2, 1, figsize=(11.0, 6.5), sharex=True)
    axes[0].plot(
        times,
        [float(row["rank_spearman"]) for row in selected],
        color="#7b2cbf",
        linewidth=2.0,
    )
    axes[0].set_ylabel("Spearman排序相关系数")
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].set_title("同一批8000条候选：理想模型排序 vs 完整响应模型排序")
    axes[1].plot(
        times,
        [100.0 * float(row["top_1pct_overlap"]) for row in selected],
        color="#00897b",
        linewidth=2.0,
        label="Top 1%候选重合率",
    )
    axes[1].plot(
        times,
        [
            float(row["ideal_best_response_percentile"])
            for row in selected
        ],
        color="#d94841",
        linewidth=1.7,
        label="理想最优候选在响应模型中的百分位",
    )
    axes[1].set_ylabel("百分比 (%)")
    axes[1].set_xlabel("时间 (s)")
    axes[1].legend(fontsize=9)
    figure.tight_layout()
    figure.savefig(
        output_dir / "candidate_ranking_audit.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def plot_monte_carlo(
    output_dir: Path, runs: list[dict[str, object]]
) -> None:
    selected = [
        row
        for row in runs
        if row["task"] == "wall_stop" and row["plant"] == "stress"
    ]
    figure, axes = plt.subplots(1, 2, figsize=(12.0, 5.0))
    labels = [MODEL_LABELS[model] for model in MODEL_LABELS]
    margins = [
        [
            100.0 * float(row["safety_margin_m"])
            for row in selected
            if row["model"] == model
        ]
        for model in MODEL_LABELS
    ]
    errors = [
        [
            1000.0 * float(row["p95_abs_first_cycle_prediction_error_m"])
            for row in selected
            if row["model"] == model
        ]
        for model in MODEL_LABELS
    ]
    boxes = axes[0].boxplot(margins, labels=labels, patch_artist=True)
    for patch, model in zip(boxes["boxes"], MODEL_LABELS):
        patch.set_facecolor(MODEL_COLORS[model])
        patch.set_alpha(0.72)
    axes[0].axhline(0.0, color="#c53030", linestyle="--")
    axes[0].set_ylabel("最小安全余量 (cm)")
    axes[0].set_title("多随机种子安全余量")
    axes[0].tick_params(axis="x", rotation=12)
    boxes = axes[1].boxplot(errors, labels=labels, patch_artist=True)
    for patch, model in zip(boxes["boxes"], MODEL_LABELS):
        patch.set_facecolor(MODEL_COLORS[model])
        patch.set_alpha(0.72)
    axes[1].set_ylabel("下一周期位置预测误差P95 (mm)")
    axes[1].set_title("多随机种子第一执行段预测误差")
    axes[1].tick_params(axis="x", rotation=12)
    figure.tight_layout()
    figure.savefig(
        output_dir / "monte_carlo_comparison.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def run_sensitivity_grid(
    seeds: int,
    controller: ControllerParams,
    smoother: SmootherParams,
) -> list[dict[str, object]]:
    wall_task = next(task for task in TASKS if task.key == "wall_stop")
    rows: list[dict[str, object]] = []
    for delay in (0.0, 0.05, 0.10):
        for tau in (0.0, 0.15, 0.30, 0.45):
            plant = PlantParams(
                key=f"d{delay:.2f}_t{tau:.2f}",
                delay_s=delay,
                tau_s=tau,
                scale=1.0,
            )
            group = []
            for seed in range(seeds):
                summary, _ = simulate_run(
                    "ideal",
                    wall_task,
                    plant,
                    seed,
                    controller,
                    smoother,
                    audit_ranking=True,
                )
                group.append(summary)
            rows.append(
                {
                    "delay_s": delay,
                    "tau_s": tau,
                    "seeds": seeds,
                    "safety_violation_rate": statistics.fmean(
                        float(row["safety_violation"]) for row in group
                    ),
                    "mean_safety_margin_m": statistics.fmean(
                        float(row["safety_margin_m"]) for row in group
                    ),
                    "mean_p95_first_cycle_prediction_error_m": statistics.fmean(
                        float(
                            row[
                                "p95_abs_first_cycle_prediction_error_m"
                            ]
                        )
                        for row in group
                    ),
                    "mean_rank_spearman": statistics.fmean(
                        float(row["mean_rank_spearman"]) for row in group
                    ),
                    "mean_top_1pct_overlap": statistics.fmean(
                        float(row["mean_top_1pct_overlap"]) for row in group
                    ),
                }
            )
    return rows


def plot_sensitivity(
    output_dir: Path, rows: list[dict[str, object]]
) -> None:
    delays = sorted({float(row["delay_s"]) for row in rows})
    taus = sorted({float(row["tau_s"]) for row in rows})
    metrics = (
        (
            "mean_p95_first_cycle_prediction_error_m",
            "下一周期位置预测误差P95 (mm)",
            1000.0,
            "magma",
        ),
        (
            "mean_rank_spearman",
            "候选代价排序Spearman相关系数",
            1.0,
            "viridis",
        ),
        (
            "mean_top_1pct_overlap",
            "Top 1%候选重合率 (%)",
            100.0,
            "cividis",
        ),
    )
    figure, axes = plt.subplots(1, 3, figsize=(15.0, 4.5))
    for axis, (metric, title, scale, cmap) in zip(axes, metrics):
        axis.grid(False)
        matrix = np.empty((len(taus), len(delays)))
        for tau_index, tau in enumerate(taus):
            for delay_index, delay in enumerate(delays):
                row = next(
                    item
                    for item in rows
                    if float(item["tau_s"]) == tau
                    and float(item["delay_s"]) == delay
                )
                matrix[tau_index, delay_index] = float(row[metric]) * scale
        image = axis.imshow(matrix, origin="lower", aspect="auto", cmap=cmap)
        axis.set_xticks(range(len(delays)), [f"{value:.2f}" for value in delays])
        axis.set_yticks(range(len(taus)), [f"{value:.2f}" for value in taus])
        axis.set_xlabel("人工延迟 (s)")
        axis.set_ylabel("人工时间常数 τ (s)")
        axis.set_title(title)
        value_min = float(np.min(matrix))
        value_span = max(float(np.max(matrix)) - value_min, 1e-12)
        for row_index in range(len(taus)):
            for column_index in range(len(delays)):
                normalized = (
                    matrix[row_index, column_index] - value_min
                ) / value_span
                axis.text(
                    column_index,
                    row_index,
                    f"{matrix[row_index, column_index]:.2f}",
                    ha="center",
                    va="center",
                    color="black" if normalized > 0.58 else "white",
                    fontsize=8,
                )
        figure.colorbar(image, ax=axis, fraction=0.046)
    figure.tight_layout()
    figure.savefig(
        output_dir / "response_sensitivity_grid.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def run_self_tests(
    controller: ControllerParams, smoother: SmootherParams
) -> None:
    assert controller.num_samples == 8000
    assert controller.horizon == 5
    assert math.isclose(controller.model_dt, 0.1)
    assert math.isclose(controller.control_dt, 0.05)
    assert smoother.feedback == "OPEN_LOOP"
    assert not smoother.scale_velocities

    internal, published = smoother_update(0.8, 0.0, smoother)
    assert math.isclose(internal, 0.05, abs_tol=1e-12)
    assert math.isclose(published, 0.05, abs_tol=1e-12)
    internal, published = smoother_update(0.8, internal, smoother)
    assert math.isclose(internal, 0.10, abs_tol=1e-12)
    assert math.isclose(published, 0.10, abs_tol=1e-12)
    internal, _ = smoother_update(0.0, internal, smoother)
    assert math.isclose(internal, 0.05, abs_tol=1e-12)

    delayed_plant = PlantParams("test", 0.10, 0.0, 1.0)
    state = PlantState(0.0, 0.0, deque([0.0, 0.0]))
    plant_update(0.4, state, delayed_plant, controller.control_dt)
    assert state.velocity == 0.0
    plant_update(0.4, state, delayed_plant, controller.control_dt)
    assert state.velocity == 0.0
    plant_update(0.4, state, delayed_plant, controller.control_dt)
    assert math.isclose(state.velocity, 0.4)

    candidates = np.full(
        (controller.num_samples, controller.horizon), 0.4
    )
    no_wall_task = TASKS[0]
    _, positions, _ = rollout_candidates(
        candidates,
        "ideal",
        0.0,
        0.0,
        0.0,
        [],
        no_wall_task,
        controller,
        smoother,
        PlantParams("ideal", 0.0, 0.0, 1.0),
    )
    assert np.allclose(positions[:, -1], 0.20)
    print("Self-tests passed: config, smoother, delay, and ideal rollout.")


def write_config_snapshot(
    output_dir: Path,
    controller: ControllerParams,
    smoother: SmootherParams,
    seeds: int,
    sensitivity_seeds: int,
) -> None:
    snapshot = {
        "controller": asdict(controller),
        "velocity_smoother": asdict(smoother),
        "artificial_plants": [asdict(plant) for plant in PLANT_SCENARIOS],
        "tasks": [asdict(task) for task in TASKS],
        "monte_carlo_seeds": seeds,
        "sensitivity_seeds": sensitivity_seeds,
        "source_files": {
            str(path.relative_to(ROOT)): sha256(path)
            for path in (
                PARAMS_PATH,
                CONTROLLER_PATH,
                KERNEL_PATH,
                REWARDS_PATH,
            )
        },
        "scope": (
            "Offline one-dimensional longitudinal projection. Artificial "
            "plant parameters; not B2, Nav2/Gazebo, or real-robot evidence."
        ),
        "excluded_as_not_applicable_in_1d": [
            "vy lateral motion",
            "omega/yaw dynamics",
            "PathAlign cost on a straight coincident path",
            "PathAngle cost on a zero-heading-error path",
            "2-D lateral bias and narrow-passage state machine",
        ],
    }
    with (output_dir / "config_snapshot.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(snapshot, stream, ensure_ascii=False, indent=2)


def print_key_results(aggregates: list[dict[str, object]]) -> None:
    print("\nFull 1-D experiment complete")
    print("Artificial response parameters are sensitivity conditions, not B2 data.")
    for plant in ("smoother_only", "mild", "stress"):
        print(f"\nwall_stop / {plant}")
        for model in MODEL_LABELS:
            row = next(
                item
                for item in aggregates
                if item["task"] == "wall_stop"
                and item["plant"] == plant
                and item["model"] == model
            )
            print(
                f"  {model:>8}: safety_margin="
                f"{100.0 * float(row['safety_margin_m_mean']):+.2f} cm, "
                f"p95_first_error="
                f"{1000.0 * float(row['p95_abs_first_cycle_prediction_error_m_mean']):.2f} mm, "
                f"violation_rate={100.0 * float(row['safety_violation_rate']):.1f}%"
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--sensitivity-seeds", type=int, default=8)
    parser.add_argument("--skip-sensitivity", action="store_true")
    parser.add_argument("--self-test-only", action="store_true")
    parser.add_argument(
        "--execution-plot-only",
        action="store_true",
        help=(
            "Read existing full_timeseries.csv and regenerate only "
            "execution_chain_bridge.png without rerunning experiments"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "results/full_1d",
    )
    args = parser.parse_args()
    if args.seeds <= 0 or args.sensitivity_seeds <= 0:
        parser.error("seed counts must be positive")

    controller = ControllerParams.load()
    smoother = SmootherParams.load()
    run_self_tests(controller, smoother)
    if args.self_test_only:
        return

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    configure_plot_style()
    if args.execution_plot_only:
        timeseries_path = output_dir / "full_timeseries.csv"
        if not timeseries_path.exists():
            parser.error(
                f"--execution-plot-only requires {timeseries_path}"
            )
        plot_execution_chain(
            output_dir,
            read_csv(timeseries_path),
            controller,
        )
        print(
            "Execution-chain bridge plot written to: "
            f"{output_dir / 'execution_chain_bridge.png'}"
        )
        return

    write_config_snapshot(
        output_dir,
        controller,
        smoother,
        args.seeds,
        args.sensitivity_seeds,
    )

    run_rows: list[dict[str, object]] = []
    time_rows: list[dict[str, object]] = []
    for task in TASKS:
        for plant in PLANT_SCENARIOS:
            for model in MODEL_LABELS:
                for seed in range(args.seeds):
                    summary, rows = simulate_run(
                        model,
                        task,
                        plant,
                        seed,
                        controller,
                        smoother,
                        audit_ranking=(model == "ideal"),
                    )
                    run_rows.append(summary)
                    time_rows.extend(rows)

    aggregates = aggregate_runs(run_rows)
    write_csv(output_dir / "monte_carlo_runs.csv", run_rows)
    write_csv(output_dir / "monte_carlo_aggregate.csv", aggregates)
    write_csv(output_dir / "full_timeseries.csv", time_rows)
    ranking_rows = [
        row
        for row in time_rows
        if row["model"] == "ideal"
    ]
    write_csv(output_dir / "candidate_ranking_audit.csv", ranking_rows)
    plot_execution_chain(output_dir, time_rows, controller)
    plot_representative(output_dir, time_rows, controller)
    plot_ranking(output_dir, time_rows)
    plot_monte_carlo(output_dir, run_rows)

    if not args.skip_sensitivity:
        sensitivity = run_sensitivity_grid(
            args.sensitivity_seeds, controller, smoother
        )
        write_csv(output_dir / "response_sensitivity_grid.csv", sensitivity)
        plot_sensitivity(output_dir, sensitivity)

    print_key_results(aggregates)
    print(f"\nResults written to: {output_dir}")


if __name__ == "__main__":
    main()

# Unitree B2 Nav2 with GPU-MPPI

[中文](README.md) · [Architecture](docs/architecture.md) · [Controllers](docs/controllers.md) · [Reproduction](docs/reproduction.md) · [Evidence](docs/experiments.md)

An end-to-end ROS 2 Humble / Nav2 workspace for Unitree B2 navigation. It integrates B2 state and high-level motion interfaces, LiDAR preprocessing, localization, global planning and a custom CUDA MPPI local controller.

The physical motion process is not started by default. It requires both the `--walk` launcher flag and `enable_motion=true` on the bridge.

## Main pipeline

```text
SportModeState -> odom + TF
PointCloud2 -> scan -> Nav2 costmaps
goal -> Navfn -> custom GPU-MPPI -> velocity smoother
     -> cmd_vel -> guarded B2 bridge -> SportClient::Move
```

## Highlights

- CPU MPPI, MPC and CUDA GPU-MPPI implementations behind the Nav2 Controller API.
- Batched rollout/cost evaluation with configurable path, obstacle, heading, speed and command-change terms.
- B2 ROS 2 state/sensor adapters and a watchdog-protected high-level motion bridge.
- Reproducible model-mismatch experiments and isolated Nav2 simulation scripts.
- Explicit evidence levels: source, build, offline test, simulation and physical validation are reported separately.

## Quick check

```bash
git clone --recurse-submodules <your-b2-repository-url> unitree-b2-nav2
cd unitree-b2-nav2
./scripts/check_source.sh
python3 experiments/mppi_model_mismatch/run_full_1d_experiment.py --self-test-only
```

See [docs/reproduction.md](docs/reproduction.md) for ROS/CUDA build instructions. Original work is Apache-2.0 licensed; third-party components retain their own licenses.

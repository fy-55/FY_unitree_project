# Unitree G1 ROS 2 Navigation

[中文](README.md) · [Architecture](docs/architecture.md) · [Reproduction](docs/reproduction.md) · [Validation](docs/validation.md)

An end-to-end ROS 2 Humble and Nav2 workspace for indoor Unitree G1 navigation. It adapts Unitree state, odometry and Mid360 point clouds to standard ROS 2 interfaces, then connects SLAM Toolbox, Nav2 planning/control, velocity smoothing, Collision Monitor and a guarded Unitree API 7105 velocity bridge.

The standard path uses Regulated Pure Pursuit. CUDA MPPI and the modular MPPI v2 package are experimental controller branches. Physical output is disabled by default with `enable_motion=false`.

## Main pipeline

```text
LowState -> joint_states -> robot TF
SportModeState -> odom -> odom/base_footprint TF
Mid360 PointCloud2 -> timestamp adapter -> LaserScan
LaserScan + odom -> SLAM Toolbox -> map/odom TF
goal -> Navfn -> RPP or GPU-MPPI -> velocity smoother
     -> Collision Monitor -> cmd_vel_safe
     -> Gazebo or guarded Unitree API 7105 bridge
```

## Highlights

- Nine ROS 2 packages covering description, state, odometry, sensors, SLAM, Nav2, control, bringup and simulation.
- Fail-closed G1 velocity bridge with finite-value checks, command and scan watchdogs, limits, forward/yaw-only policy and zero-on-exit behavior.
- Gazebo factory world, simulated Mid360, planar motion proxy, serialized SLAM map and RViz configuration.
- Stable RPP baseline plus CPU/GPU MPPI and a modular controller research branch.
- Explicit separation of source availability, local build, simulation evidence and physical-robot validation.

## Quick start

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-skip nav2_custom_plugins nav2_custom_plugins_v2
source src/g1_nav_sim/env/use_g1_sim.sh
ros2 launch g1_nav_sim g1_nav_sim.launch.py
```

Use the commands in [docs/reproduction.md](docs/reproduction.md) to start localization and Nav2 in separate terminals. Real-robot startup is intentionally not a one-command workflow; follow [docs/real_robot_safety.md](docs/real_robot_safety.md).

Original work is Apache-2.0 licensed. Third-party models and Unitree interfaces keep their own licenses; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

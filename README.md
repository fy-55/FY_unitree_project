# Unitree B2 Nav2 with GPU-MPPI

[English](README_EN.md) · [架构](docs/architecture.md) · [控制器](docs/controllers.md) · [复现流程](docs/reproduction.md) · [实验与证据](docs/experiments.md) · [真机安全](docs/real_robot_safety.md)

面向 Unitree B2 的 ROS 2 Humble / Nav2 全流程导航项目，重点实现 B2 状态与运动接口、激光雷达预处理、Nav2 规划控制链，以及可插拔的 CPU/GPU MPPI 控制器。仓库同时保留模型失配研究、隔离仿真脚本和可复现实验报告。

> 这是个人研究与工程项目，不是 Unitree 官方产品。Unitree 是其权利人的商标。公开版默认不启动真机运动。

## 3 分钟了解项目

```text
B2 SportModeState
  -> b2_pub
  -> /odom + odom/base TF

RoboSense PointCloud2
  -> relay / PointCloud-to-LaserScan
  -> /converted_scan
  -> local/global costmap

Nav2 goal
  -> Navfn global planner
  -> FollowPath = custom CUDA MPPI
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel
  -> b2_walk safety gate
  -> Unitree SportClient::Move()
```

## 我完成的工作

- 打通 B2 的状态、里程计、TF、激光雷达、Nav2 与高层运动接口。
- 实现 MPC、CPU-MPPI 与 CUDA GPU-MPPI，并通过 `pluginlib` 接入 Nav2 Controller Server。
- 在 GPU 上并行执行采样、rollout 和代价评估，支持路径、障碍、航向、速度、控制变化与安全走廊等代价。
- 实现控制器统计日志、轨迹可视化、恢复行为与诊断原型。
- 为 B2 运动出口增加显式 `enable_motion`、有限值检查、限幅、deadband 和超时 `StopMove()`。
- 建立 TurtleBot3 隔离仿真、B2 全向代理验证和一维模型失配实验，保留成功、失败与尚未验证的边界。

## 控制器装载链

```text
controller_server
  -> FollowPath.plugin = nav2_custom_plugins/MPPIGPUController
  -> ament resource index
  -> plugins.xml
  -> libnav2_custom_plugins_gpu.so
  -> MPPIGPUController::computeVelocityCommands()
```

当前 B2 参数基线为 20 Hz 控制频率、8000 条候选、预测 5 步、`dt=0.1 s`。这些是配置事实，不代表真实 B2 的响应延迟或动力学已经辨识。

## 快速开始：先做离线验证

```bash
git clone --recurse-submodules <your-b2-repository-url> unitree-b2-nav2
cd unitree-b2-nav2

./scripts/check_source.sh
python3 experiments/mppi_model_mismatch/run_full_1d_experiment.py --self-test-only
```

ROS 2/CUDA 构建：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to b2_driver b2_navigation nav2_custom_plugins \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89
```

将 `89` 改为目标 NVIDIA GPU 的 compute capability。完整环境和仿真步骤见[复现流程](docs/reproduction.md)。

## 模块

| 模块 | 功能 | 关键输出 |
|---|---|---|
| `b2_driver` | B2 状态、TF、雷达 relay、运动桥、调试工具 | `/odom`、TF、点云、SportClient |
| `b2_navigation` | Nav2/SLAM launch、地图与参数 | planner/controller/costmap 生命周期 |
| `nav2_custom_plugins` | MPC、CPU-MPPI、GPU-MPPI、恢复与诊断原型 | Nav2 Controller/Behavior 插件 |
| `experiments/mppi_model_mismatch` | 理想/延迟/滞后/死区模型的离线审计 | CSV、图、阶段报告 |
| `experiments/*validation` | 隔离 Nav2 与全向代理场景 | 运行脚本、配置快照、汇总 |
| `src/unitree_ros2` | 官方 Unitree ROS 2 依赖，Git submodule | `unitree_go`、`unitree_api` |

## 当前证据边界

| 层级 | 状态 |
|---|---|
| 源码与插件链 | 已具备，GPU-MPPI 是当前配置的 `FollowPath` |
| 本机编译 | 2026-08-27：独立临时目录中 5 个相关包构建通过 |
| 自动化检查 | 15 项 lint/XML 测试通过；源码检查和模型失配自检通过 |
| 隔离仿真 | 有成功与失败运行，必须按单次日志判断 |
| 真实 B2 参数辨识 | 尚未完成 |
| 真机导航收益/论文结论 | 当前不能声称已验证 |

完整矩阵见[实验与证据](docs/experiments.md)。真实场地地图、rosbag、完整运行日志和构建产物不会公开。

## 许可证

本人原创部分采用 [Apache-2.0](LICENSE)。Unitree 示例接口和 nlohmann/json 保留原许可证，见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

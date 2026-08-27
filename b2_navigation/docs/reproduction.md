# 从零复现

## 1. 环境与依赖

- Ubuntu 22.04
- ROS 2 Humble Desktop + Nav2 + SLAM Toolbox
- NVIDIA GPU、驱动和 CUDA Toolkit（构建 GPU-MPPI）
- `colcon`、`rosdep`、`vcs`、CycloneDDS

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions python3-rosdep python3-vcstool python3-yaml \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan \
  ros-humble-rmw-cyclonedds-cpp libyaml-cpp-dev
```

## 2. 克隆

```bash
git clone --recurse-submodules <your-b2-repository-url> unitree-b2-nav2
cd unitree-b2-nav2
```

`src/unitree_ros2` 是固定版本的官方依赖。如果首次克隆漏了 submodule：

```bash
git submodule update --init --recursive
```

## 3. 无 ROS/CUDA 的最小验收

```bash
./scripts/check_source.sh
python3 experiments/mppi_model_mismatch/run_full_1d_experiment.py --self-test-only
```

预期最后一行：

```text
Self-tests passed: config, smoother, delay, and ideal rollout.
```

这只能证明实验配置和数学辅助函数的内部一致性，不是真机证据。

## 4. ROS 2/CUDA 构建

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-up-to b2_driver b2_navigation nav2_custom_plugins \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89
```

将 `89` 改为目标 GPU 的 compute capability。验收：

```bash
source install/setup.bash
ros2 pkg prefix b2_driver
ros2 pkg prefix b2_navigation
ros2 pkg prefix nav2_custom_plugins
ros2 plugin list | grep nav2_custom_plugins
```

本次发布在 Ubuntu 22.04、ROS 2 Humble、GCC 11.4、RTX 4060 Laptop GPU（compute capability 8.9）与 CUDA 13.2 上，使用独立临时构建目录验证：5 个相关包完成构建，15 项 lint/XML 测试通过。编译器仍会报告部分历史 warning；这不等于仿真或真机导航已经验证。

## 5. 隔离仿真

仓库的 `experiments/nav2_official_mppi_sim` 和 `experiments/custom_mppi_nav2_validation` 提供独立脚本。先阅读对应 README，再执行：

```bash
cd experiments/custom_mppi_nav2_validation
./scripts/check_isolation.sh
./scripts/start_sim.sh
```

隔离环境固定 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`，避免连接真实机器人。每次运行以当次日志和结果文件为准；旧目录中既有成功也有失败记录。

## 6. 真机前无运动启动

```bash
./scripts/start_b2_navigation.sh
```

无参数默认只启动传感器和扫描转换，既不加载场地地图，也不启动运动桥。先验收：

```bash
ros2 topic hz /odom
ros2 topic hz /converted_scan
ros2 run tf2_ros tf2_echo odom base
```

准备好自己的 SLAM Toolbox 序列化地图（`map.posegraph` 和 `map.data`）后，显式提供不带扩展名的绝对路径：

```bash
./scripts/start_b2_navigation.sh --map-base=/absolute/path/to/map
```

此时脚本启动定位和 Nav2，但仍不启动 `b2_walk`，可继续检查 `/plan` 和 `/cmd_vel`。只有按[真机安全流程](real_robot_safety.md)完成检查后，才允许在同一地图参数后追加 `--walk`。

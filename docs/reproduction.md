# 从零复现

本页先给出不连接机器人的可验证流程。每一步都列出输入、输出和验收标准。

## 1. 环境

- Ubuntu 22.04
- ROS 2 Humble Desktop
- Nav2、SLAM Toolbox、Gazebo Classic 11
- `rosdep`、`colcon`、CycloneDDS RMW
- 可选：NVIDIA CUDA Toolkit（仅构建自定义 GPU 控制器）

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs \
  ros-humble-pointcloud-to-laserscan ros-humble-laser-filters \
  ros-humble-rmw-cyclonedds-cpp
```

## 2. 获取和构建标准 RPP 路径

```bash
git clone <your-g1-repository-url> unitree-g1-nav2
cd unitree-g1-nav2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-skip nav2_custom_plugins nav2_custom_plugins_v2
```

输入是仓库源码与系统 ROS 包；输出是 `install/`。成功标准是 colcon 没有失败包。

若需要 GPU 控制器：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select nav2_custom_plugins nav2_custom_plugins_v2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89
```

将 `89` 改为目标 GPU 的 compute capability。没有 CUDA 时不要构建这两个包。

## 3. 静态检查

```bash
./scripts/check_source.sh
```

它检查 Python 语法、Shell 语法、XML、YAML，以及真机运动默认门控。它不替代 ROS 2 构建或运行测试。

## 4. 启动仿真输入层

终端 A：

```bash
source src/g1_nav_sim/env/use_g1_sim.sh
ros2 launch g1_nav_sim g1_nav_sim.launch.py
```

输出应包含 Gazebo、G1 模型、`/odom`、TF、点云和 `/scan`。另开终端验收：

```bash
source src/g1_nav_sim/env/use_g1_sim.sh
ros2 topic hz /odom
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
```

成功标准：两个话题持续发布，TF 可连续查询。

## 5. 启动定位

终端 B：

```bash
source src/g1_nav_sim/env/use_g1_sim.sh
SIM_MAP_DIR="$(ros2 pkg prefix --share g1_nav_sim)/maps"
SLAM_CONFIG="$(ros2 pkg prefix --share g1_nav_slam)/config/localization_sim.yaml"
ros2 launch g1_nav_slam localization.launch.py \
  map_name:=sim_g1 map_dir:="$SIM_MAP_DIR" \
  params_file:="$SLAM_CONFIG" use_sim_time:=true
```

验收：

```bash
ros2 topic echo /map --once --no-arr
ros2 run tf2_ros tf2_echo map odom
```

## 6. 启动 Nav2

终端 C：

```bash
source src/g1_nav_sim/env/use_g1_sim.sh
ros2 launch g1_nav_nav2 navigation.launch.py use_sim_time:=true
```

验收：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /collision_monitor
ros2 topic info /cmd_vel_safe
```

三个 lifecycle 节点都应为 `active [3]`。在 RViz 发目标后，可以按顺序观察：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_safe
```

## 7. 常见问题

| 现象 | 优先检查 |
|---|---|
| 找不到 ROS launch 模块 | 是否先 `source /opt/ros/humble/setup.bash` |
| 找不到本项目包 | 是否构建并加载 `install/setup.bash` |
| 仿真看见真机话题 | `ROS_DOMAIN_ID=80`、`ROS_LOCALHOST_ONLY=1` 是否生效 |
| 没有 `map->odom` | 定位进程、四个 map 文件、`use_sim_time` |
| Nav2 lifecycle 不 active | 控制器插件、参数文件、TF、`/scan` 和 `/odom` |
| 无 CUDA/NVCC | 使用 RPP 构建命令跳过两个 GPU 包 |

真实 G1 只允许在上述链路通过后进入[分阶段真机流程](real_robot_safety.md)。

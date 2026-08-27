# Unitree G1 ROS 2 Navigation

[English](README_EN.md) · [架构](docs/architecture.md) · [复现流程](docs/reproduction.md) · [验证状态](docs/validation.md) · [真机安全](docs/real_robot_safety.md)

基于 ROS 2 Humble 与 Nav2 的 Unitree G1 室内导航工作区。项目覆盖从机器人状态、点云和里程计适配，到 SLAM、全局规划、局部控制、碰撞监控以及 Unitree 高层速度接口的完整链路；默认配置为无运动模式，优先支持离线检查与 Gazebo 复现。

> 这是个人研究与工程项目，不是 Unitree 官方产品。Unitree 是其权利人的商标。

## 3 分钟了解项目

```text
G1 LowState ───────────────> /joint_states ──> robot_state_publisher
SportModeState ────────────> /odom + odom->base_footprint
Mid360 PointCloud2 ────────> 时间戳适配 ──> /scan
                                      │
                                      v
                     slam_toolbox: map->odom
                                      │
                                      v
Navfn global planner ──> RPP / GPU-MPPI ──> velocity_smoother
                                      │
                                      v
                     Collision Monitor: /cmd_vel_safe
                                      │
                    ┌─────────────────┴─────────────────┐
                    v                                   v
              Gazebo 平面底盘                  G1 API 7105 速度桥
                                               默认 enable_motion=false
```

默认真机控制器使用稳定、易解释的 Regulated Pure Pursuit（RPP）。GPU-MPPI 和模块化 MPPI v2 作为控制器研究分支保留，不作为首次真机启动项。

## 我完成的工作

- 将 G1 原始 DDS/ROS 2 话题适配成 Nav2 需要的 `/joint_states`、`/odom`、TF 与 `/scan`。
- 建立 `slam_toolbox` 建图/定位链，并避免与 AMCL 同时发布 `map -> odom`。
- 配置 Navfn、RPP、BT Navigator、局部/全局 costmap、速度平滑与 Collision Monitor。
- 实现 G1 API 7105 速度桥：只允许前向和偏航、NaN/Inf 拒绝、指令/激光超时置零、速度限幅与退出置零。
- 构建 Gazebo 工厂环境、G1 导航代理、虚拟 Mid360、仿真地图和 RViz 配置。
- 维护 CPU/GPU MPPI、恢复行为和模块化 MPPI v2，用于控制器替换与实验扩展。

## 快速复现：仿真优先

环境基线：Ubuntu 22.04、ROS 2 Humble、Nav2、Gazebo Classic 11。标准 RPP 仿真不要求 CUDA。

```bash
git clone <your-g1-repository-url> unitree-g1-nav2
cd unitree-g1-nav2

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-skip nav2_custom_plugins nav2_custom_plugins_v2

source src/g1_nav_sim/env/use_g1_sim.sh
ros2 launch g1_nav_sim g1_nav_sim.launch.py
```

另开终端加载同一仿真环境，然后启动定位和 Nav2：

```bash
source src/g1_nav_sim/env/use_g1_sim.sh
SIM_MAP_DIR="$(ros2 pkg prefix --share g1_nav_sim)/maps"
ros2 launch g1_nav_slam localization.launch.py \
  map_name:=sim_g1 map_dir:="$SIM_MAP_DIR" \
  params_file:="$(ros2 pkg prefix --share g1_nav_slam)/config/localization_sim.yaml" \
  use_sim_time:=true
```

```bash
source src/g1_nav_sim/env/use_g1_sim.sh
ros2 launch g1_nav_nav2 navigation.launch.py use_sim_time:=true
```

完整依赖、验收命令和故障排查见[复现流程](docs/reproduction.md)。

## 模块

| 包 | 作用 | 主要输入 → 输出 |
|---|---|---|
| `g1_nav_description` | G1 URDF、mesh、TF 与 RViz | URDF + `/joint_states` → TF |
| `g1_nav_state` | 关节状态适配 | `/lf/lowstate` → `/joint_states` |
| `g1_nav_odometry` | 里程计与平面 TF | `SportModeState` → `/odom` + TF |
| `g1_nav_sensors` | 点云时间戳、二维激光与滤波 | PointCloud2 → `/scan` |
| `g1_nav_slam` | 建图与序列化地图定位 | `/scan` + `/odom` → `/map` + `map->odom` |
| `g1_nav_nav2` | 规划、控制、BT、costmap、碰撞监控 | goal → `/cmd_vel_safe` |
| `g1_nav_control` | 安全门控与 Unitree API 适配 | `/cmd_vel_safe` + `/scan` → API 7105 |
| `g1_nav_sim` | Gazebo 与传感器/底盘仿真 | `/cmd_vel_safe` ↔ `/odom`, `/scan` |
| `nav2_custom_plugins*` | CPU/GPU MPPI 与控制器原型 | Nav2 path/costmap → Twist |

更详细的边界和扩展点见[模块说明](docs/modules.md)。

## 控制器与扩展能力

- `nav2.yaml`：默认 RPP 基线，适合先证明整条导航链可靠。
- `nav2_mppi.yaml`：GPU-MPPI 实验配置，支持采样规模、预测时域、代价项和日志扩展。
- `nav2_custom_plugins_v2`：将参数、路径管理、GPU 上传、代价评估、状态机、后处理和可视化解耦，便于增加 critic 或替换动力学模型。
- 控制器通过 Nav2 `pluginlib` 接口接入，规划器、控制器、恢复行为和进度检测器可以独立替换。

## 当前证据边界

| 层级 | 状态 |
|---|---|
| 源码与配置 | 已具备完整模块 |
| 本机编译 | 2026-08-27：13 个相关包完成 Release 构建 |
| 仿真流程 | 已提供 Gazebo/SLAM/Nav2 启动与验收步骤 |
| 自动化测试 | 110 项 lint/静态测试：0 失败、6 跳过；源码检查通过 |
| 真机 | 默认无运动；不能仅凭代码或本机构建声称真机导航已验证 |

详细说明见[验证状态](docs/validation.md)。真实实验室地图、rosbag、构建产物和许可证不明确的本地 `unitree_slam` 示例不会进入公开仓库。

## 许可证

本人原创部分采用 [Apache-2.0](LICENSE)。机器人模型、Unitree 消息和其他依赖保留各自许可证，见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

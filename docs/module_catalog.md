# 功能包与模块目录

本目录以当前 `package.xml`、CMake target、launch、参数文件和 `plugins.xml` 为依据。模块被分为四类：

- **主链**：正常导航流程会使用。
- **支撑**：模型、仿真、消息或统一启动，不直接生成导航控制量。
- **研究**：控制器或实验分支，用于算法研究和离线分析。
- **测试/旧链**：用于单节点验证或早期方案，不属于当前 Nav2 主链。

## G1 功能包

### `g1_nav_bringup` — 真机只读输入总入口

- 类型：主链/统一启动。
- 入口：`launch/real_robot_pre_slam.launch.py`。
- 组合：G1 模型、`joint_state_bridge`、SportModeState 里程计、TF 桥、点云转二维扫描。
- 输入：`/lf/lowstate`、`/odommodestate`、`/utlidar/cloud_livox_mid360`。
- 输出：`/joint_states`、`/odom`、`/scan` 和机器人/里程计 TF。
- 边界：只建立 SLAM 前的数据链，不启动 Nav2 运动桥，不发送 API 7105。

### `g1_nav_description` — 机器人模型与内部 TF

- 类型：支撑。
- 入口：`launch/model.launch.py`。
- 内容：G1 URDF/Xacro、mesh、RViz 配置和模型许可证。
- 输入：`/joint_states`。
- 输出：`robot_description`、`/tf`、`/tf_static`。
- 边界：描述机器人几何与关节树，不控制关节或步态。

### `g1_nav_state` — Unitree 低层状态适配

- 类型：主链。
- 可执行文件：`joint_state_bridge`。
- 数据契约：`unitree_hg/LowState` `/lf/lowstate` → `sensor_msgs/JointState` `/joint_states`。
- 作用：把 29 个机身关节的 Unitree 状态映射为 ROS 标准关节状态，供模型 TF 和 RViz 使用。
- 边界：只读；Dex3 手指状态不混入机身 29 关节映射。

### `g1_nav_odometry` — 里程计标准化与 TF 责任分离

- 类型：主链，并含离线测试工具。
- `sportmode_odom_adapter`：`/odommodestate` → `/g1/internal_odom`，处理位置、姿态、速度坐标系和 `base_link` 关系。
- `odom_tf_bridge`：`/g1/internal_odom` → `/odom`，发布 `odom -> base_footprint`。
- `fake_odom_publisher`：生成受控假里程计，只用于离线 TF 测试。
- 入口：`launch/offline_odom_tf_test.launch.py` 可独立验证 TF，不需要连接机器人。
- 边界：提供局部运动估计，不等于全局真值；`map -> odom` 由 SLAM Toolbox 负责。

### `g1_nav_sensors` — Mid360 点云到 Nav2 激光扫描

- 类型：主链。
- 自研可执行文件：`pointcloud_timestamp_adapter`。
- 组合节点：`pointcloud_to_laserscan_node`、`laser_filters/scan_to_scan_filter_chain`。
- 数据链：`/utlidar/cloud_livox_mid360` → `/g1/cloud_synced` → `/scan_raw` → `/scan`。
- 作用：修正/监控点云时间戳、转换坐标系、按高度切片为 360° LaserScan，再去除散斑。
- 边界：只做感知预处理，不做 SLAM、costmap 或运动决策。

### `g1_nav_slam` — 建图、序列化地图与定位

- 类型：主链。
- `mapping.launch.py`：启动 `async_slam_toolbox_node`，在线建立地图。
- `localization.launch.py`：校验地图文件后启动 `localization_slam_toolbox_node`。
- `save_map.sh`：同时保存占据栅格和 SLAM Toolbox 序列化状态。
- 输入：`/scan`、`/odom` 和 TF。
- 输出：`/map`、`map -> odom`，以及 `.pgm/.yaml/.posegraph/.data` 地图文件。
- 边界：定位模式下它是唯一 `map -> odom` 发布者；真实场地图属于运行环境数据。

### `g1_nav_nav2` — 任务规划、局部控制和碰撞监控

- 类型：主链。
- 入口：`launch/navigation.launch.py`。
- 主要组件：BT Navigator、Navfn Planner、Controller Server、local/global costmap、Velocity Smoother、Waypoint Follower、Collision Monitor。
- 默认控制器：`nav2.yaml` 中的 Regulated Pure Pursuit（RPP）。
- 研究控制器：`nav2_mppi.yaml` 中的 `nav2_custom_plugins/MPPIGPUController`。
- 速度链：Controller Server `/cmd_vel_nav` → Velocity Smoother `/cmd_vel` → Collision Monitor `/cmd_vel_safe`。
- 辅助节点：`static_footprint_publisher` 为 Collision Monitor 提供固定安全足迹。
- 边界：启动 Nav2 后仍停在 `/cmd_vel_safe`，不会自动连接 G1 API。

### `g1_nav_control` — Nav2 到 G1 高层运动 API 的最终安全门

- 类型：主链，但默认关闭物理输出。
- 可执行文件：`g1_velocity_bridge`；入口 `launch/velocity_bridge.launch.py`。
- 输入：`/cmd_vel_safe`、`/scan`。
- 输出：限幅后诊断话题 `/cmd_vel_g1`；仅在 `enable_motion=true` 时发布 `/api/sport/request` API 7105。
- 安全机制：只允许前向和偏航、NaN/Inf 拒绝、命令/扫描超时置零、速度限幅、短有效期和停机置零。
- 边界：调用 G1 已有高层运动服务，不实现双足步态或关节力矩控制。

### `g1_nav_sim` — G1 导航集成仿真

- 类型：支撑/仿真。
- 入口：`launch/g1_nav_sim.launch.py`。
- 内容：Gazebo world、工厂场景、平面运动代理、虚拟 Mid360、仿真地图、RViz 与隔离 DDS 环境脚本。
- 输出契约：与真机相同的 `/odom`、TF、点云和 `/scan`，消费 `/cmd_vel_safe`。
- 作用：验证 SLAM、Nav2、控制话题和 TF 是否闭环。
- 边界：平面代理不模拟 G1 双足动力学、打滑、平衡或真实刹停距离。

### `nav2_custom_plugins` — 第一代 MPC/MPPI 插件集合

- 类型：研究。
- `nav2_custom_plugins_core` 动态库：
  - `MPCController`：确定性 MPC 基线。
  - `MPPIController`：CPU 蒙特卡洛采样控制器。
  - `EscapeObstacle`：恢复行为原型。
- `nav2_custom_plugins_gpu` 动态库：`MPPIGPUController`，在 CUDA 上并行 rollout 和计算代价。
- 工具：`test_plugin` 检查 pluginlib 装载；`costmap_reader_tool` 读取/可视化 costmap。
- 装载链：YAML 类名 → `plugins.xml` → 共享库 → `PLUGINLIB_EXPORT_CLASS`。
- 边界：`AdaptiveProgressChecker` 仅保留源码，当前未进入构建/注册，不能作为可用插件宣传。

### `nav2_custom_plugins_v2` — 模块化 GPU-MPPI 研究分支

- 类型：研究原型，不是默认真机控制器。
- 对外插件：`nav2_custom_plugins_v2/MPPISteeringController`。
- 模块：Nav2 adapter、参数加载、Path Manager、MPPI Core/Pipeline、GPU Engine/Uploader、Critic Manager、状态机、速度后处理、RViz 可视化。
- 消息：`VelocitySteering.msg` 表示解耦速度/转向控制结果。
- 测试目标：`test_critics`、`benchmark_mppi`。
- 作用：降低单体控制器耦合，便于替换 critic、控制空间和动力学模型。
- 边界：构建成功不代表已在 G1 真机证明性能或安全收益。

### `unitree_api`、`unitree_go`、`unitree_hg` — Unitree ROS 消息接口

- 类型：外部接口依赖，不计作自研导航算法。
- `unitree_api`：API request/response 消息，G1 API 7105 与 B2 SportClient 都依赖它。
- `unitree_go`：SportModeState 等 Go2/B2 高层状态消息。
- `unitree_hg`：G1/H1 低层状态、关节和手部消息。

## B2 功能包

### `b2_driver` — B2 状态、雷达、运动和单节点工具集合

- 类型：主链 + 测试/旧链。
- 依赖：Unitree `unitree_go`/`unitree_api` 消息和高层 SportClient request 封装。

| 可执行文件 | 分类 | 输入 → 输出 | 作用与边界 |
|---|---|---|---|
| `b2_pub` | 主链 | `lf/sportmodestate` → `/odom` + `odom -> base` | 将 B2 高层状态转换为 Nav2 里程计与 TF |
| `rslidar_relay` | 主链 | `/rslidar_points` → `/rslidar` | 更新时间戳并过滤无效点、机身附近点和距离异常点 |
| `b2_walk` | 主链/安全出口 | `/cmd_vel`（可配置）→ `SportClient::Move/StopMove` | 默认 `enable_motion=false`；有限值检查、deadband、限幅和 1 s watchdog |
| `b2_sub` | 诊断 | `/odom` → 终端日志 | 检查里程计内容，不参与控制 |
| `rslidar_simple_relay` | 兼容工具 | 点云/扫描 relay | 简单转发版本，不是标准启动脚本的主处理器 |
| `b2_move_test` | 危险测试 | 固定测试参数 → SportClient | 可直接调用真实运动 API；不得作为快速复现步骤 |
| `b2_keyboard_cmd_vel` | 手动测试 | 键盘 → `/b2/b2cmd_vel` | 早期遥控/停止工具，不是实体急停 |
| `b2_path_generator` | 测试/旧链 | 参数 → `/plan` | 生成直线、方形或圆形测试路径 |
| `b2_obstacle_detector` | 测试/旧链 | `/converted_scan` + `/odom` → `/obstacle_ahead` | 基于运动方向的规则障碍检测 |
| `b2_path_follower` | 测试/旧链 | `/plan` + TF + `/obstacle_ahead` → `/b2/b2cmd_vel` | 独立 Pure Pursuit 早期链，不是当前 Nav2 GPU-MPPI `FollowPath` |

标准启动只使用 `b2_pub`、`rslidar_relay` 和按需显式开启的 `b2_walk`；其他工具不会被 `start_b2_navigation.sh` 自动启动。

### `b2_navigation` — B2 SLAM/Nav2 组合与参数

- 类型：主链。
- `b2_localization.launch.py`：用显式 `.posegraph/.data` 地图基名启动 SLAM Toolbox localization。
- `pcl_to_scan.launch.py`：`/rslidar` PointCloud2 → `/converted_scan` LaserScan。
- `b2_nav2_launch.py`：加载 Nav2 navigation servers 和 `dog_nav_params.yaml`。
- 主配置：20 Hz Controller Server、Navfn、GPU-MPPI、local/global costmap、Velocity Smoother。
- 速度链：Controller Server `/cmd_vel_nav` → Velocity Smoother `/cmd_vel` → `b2_walk`。
- 重要边界：参数文件虽然含 `collision_monitor` 段，但当前 launch 没有启动该节点，且 `FootprintApproach.enabled=false`；不能把它写成已生效的 B2 安全层。
- 地图策略：公开仓库只保留地图使用说明，真实场地图由使用者在启动时提供。

### `nav2_custom_plugins` — B2 CPU/GPU 控制器与恢复原型

- 类型：主控制器 + 研究模块。
- `MPCController`：确定性基线。
- `MPPIController`：CPU 采样基线。
- `MPPIGPUController`：当前 `dog_nav_params.yaml` 的 `FollowPath`，CUDA 并行生成/评估候选轨迹。
- `EscapeObstacle`：Nav2 recovery behavior 原型。
- 输入：机器人位姿、速度、全局路径、local costmap、TF。
- 输出：`TwistStamped` 控制命令、候选/最优轨迹可视化和可选统计日志。
- 当前配置事实：20 Hz、8000 candidates、horizon 5、`dt=0.1 s`；这些不是 B2 实测响应参数。
- 边界：`AdaptiveProgressChecker` 源码保留但没有构建/注册。

### `b2_navigation/src/unitree_ros2` — 官方外部依赖

- 类型：Git submodule/第三方依赖。
- 固定 commit：`5204e6e098ee53f4bd929bd77eb1d387cd0fa842`。
- 提供：Unitree ROS 2 消息、SDK 示例和相关接口构建基础。
- 边界：不计作本人原创工作；本机 submodule 内的 build/install 不进入 portfolio。

## B2 实验模块

| 目录 | 作用 | 当前证据边界 |
|---|---|---|
| `mppi_model_mismatch` | 延迟、滞后、死区、速度缩放下的一维 rollout/闭环/候选排序审计 | 可证明代码对假设失配的响应，不能反推真实 B2 动力学 |
| `nav2_official_mppi_sim` | 官方 Nav2 MPPI + TurtleBot3 隔离基线 | 验证标准 Nav2 仿真链，不能代表 B2 |
| `custom_mppi_nav2_validation` | 自定义 GPU-MPPI 的 pluginlib 装载和同类场景验证 | 证明插件能否运行；结果需按单次日志判断 |
| `custom_mppi_b2_omni_validation` | `vx/vy/wz` 全向代理和横移场景 | 更接近接口形态，但不是 B2 真实动力学 |
| `b2_execution_risk_mppi_paper` | 假设、里程碑、指标和停止条件 | 研究计划，不是已完成论文或创新结论 |

## 哪些内容最能体现个人工作

作品集评审时，应优先阅读以下链路，而不是把第三方消息包数量当作工作量：

1. G1：`g1_nav_bringup` → `g1_nav_odometry`/`g1_nav_sensors` → `g1_nav_slam` → `g1_nav_nav2` → `g1_nav_control`。
2. B2：`b2_driver` 数据适配 → `b2_navigation` → `nav2_custom_plugins/MPPIGPUController` → `b2_walk`。
3. 控制器研究：GPU rollout、代价组合、pluginlib 接入、模型失配实验和证据边界。
4. 工程能力：TF 责任拆分、可移植启动、无运动默认、依赖/许可证整理和可复现检查。

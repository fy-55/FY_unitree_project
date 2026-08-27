# 系统流程与架构图

## 1. 两个项目的共同分层

```mermaid
flowchart TB
  A[机器人原始状态与传感器] --> B[ROS 2 接口适配层]
  B --> C[标准 scan / odom / TF]
  C --> D[SLAM 或地图定位]
  D --> E[Nav2 BT 与全局规划]
  E --> F[局部控制器]
  C --> F
  F --> G[速度平滑/碰撞或执行安全层]
  G --> H[Unitree 高层运动接口]
  H --> A

  I[离线检查/仿真/实验] -.验证各层.-> B
  I -.验证各层.-> F
  I -.验证各层.-> G
```

两套导航工程采用相似的软件分层，但拥有各自的 ROS 包、参数和运行环境；语音交互项目则是独立的 C++ 应用。

## 2. G1 完整导航流程

```mermaid
flowchart LR
  subgraph RobotInput[G1 输入]
    LOW[/lf/lowstate/]
    SPORT[/odommodestate/]
    MID[/utlidar/cloud_livox_mid360/]
  end

  subgraph Adapt[G1 标准化]
    STATE[g1_nav_state<br/>joint_state_bridge]
    ODOM1[g1_nav_odometry<br/>sportmode_odom_adapter]
    ODOM2[g1_nav_odometry<br/>odom_tf_bridge]
    PC[g1_nav_sensors<br/>timestamp adapter]
    SCAN[PointCloud-to-LaserScan<br/>+ speckle filter]
  end

  subgraph Localization[定位]
    SLAM[g1_nav_slam<br/>SLAM Toolbox]
  end

  subgraph Navigation[Nav2]
    BT[BT Navigator]
    PLAN[Navfn Planner]
    CTRL[RPP default<br/>or GPU-MPPI]
    COST[Local/Global Costmap]
    SMOOTH[Velocity Smoother]
    COLLISION[Collision Monitor]
  end

  subgraph Execute[执行边界]
    GATE[g1_nav_control<br/>limits + watchdogs]
    API[Unitree API 7105]
  end

  LOW --> STATE --> JOINT[/joint_states/]
  JOINT --> TF[robot_state_publisher TF]
  SPORT --> ODOM1 --> INTERNAL[/g1/internal_odom/]
  INTERNAL --> ODOM2 --> ODOM[/odom + odom→base_footprint/]
  MID --> PC --> SCAN --> LASER[/scan/]
  ODOM --> SLAM
  LASER --> SLAM
  SLAM --> MAP[/map + map→odom/]
  MAP --> BT --> PLAN --> CTRL
  ODOM --> CTRL
  LASER --> COST --> CTRL
  CTRL -->|/cmd_vel_nav| SMOOTH -->|/cmd_vel| COLLISION
  LASER --> COLLISION
  COLLISION -->|/cmd_vel_safe| GATE
  GATE -->|enable_motion=true only| API
```

### G1 TF 责任

```mermaid
flowchart LR
  MAP[map] -->|SLAM Toolbox| ODOM[odom]
  ODOM -->|g1_nav_odometry or Gazebo| FOOT[base_footprint]
  FOOT -->|fixed height transform| BASE[base_link]
  BASE -->|robot_state_publisher| LINKS[G1 links and sensors]
```

- `map -> odom`：SLAM Toolbox 唯一负责。
- `odom -> base_footprint`：真机由里程计适配器负责，仿真由 Gazebo 负责。
- `base_link -> robot links`：URDF + `/joint_states` 负责。

### G1 启动阶段

```mermaid
sequenceDiagram
  participant O as Operator
  participant B as g1_nav_bringup
  participant S as g1_nav_slam
  participant N as g1_nav_nav2
  participant C as g1_nav_control
  participant R as Physical G1

  O->>B: 启动只读输入链
  B-->>O: 验证 joint_states / odom / scan / TF
  O->>S: 加载地图并定位
  S-->>O: 验证 map->odom
  O->>N: 启动 Nav2 + Collision Monitor
  N-->>O: 观察 plan 和 cmd_vel_safe
  O->>C: enable_motion=false
  C-->>O: 只显示限幅后诊断输出
  O->>C: 现场检查后显式 enable_motion=true
  C->>R: API 7105 有效期速度请求
```

## 3. B2 完整导航流程

```mermaid
flowchart LR
  subgraph RobotInput[B2 输入]
    SPORT[lf/sportmodestate]
    LIDAR[/rslidar_points/]
  end

  subgraph Adapt[B2 数据适配]
    PUB[b2_driver/b2_pub]
    RELAY[b2_driver/rslidar_relay]
    P2S[pointcloud_to_laserscan]
  end

  subgraph Localization[B2 定位]
    SLAM[b2_localization.launch<br/>SLAM Toolbox]
  end

  subgraph Navigation[B2 Nav2]
    BT[BT Navigator]
    PLAN[Navfn Planner]
    COST[Local/Global Costmap]
    MPPI[MPPIGPUController]
    SMOOTH[Velocity Smoother]
  end

  subgraph Execute[B2 执行边界]
    WALK[b2_walk<br/>enable gate + clamp + watchdog]
    SPORTAPI[SportClient Move/StopMove]
  end

  SPORT --> PUB --> ODOM[/odom + odom→base/]
  LIDAR --> RELAY --> CLOUD[/rslidar/]
  CLOUD --> P2S --> SCAN[/converted_scan/]
  ODOM --> SLAM
  SCAN --> SLAM
  SLAM --> MAP[/map + map→odom/]
  MAP --> BT --> PLAN --> MPPI
  ODOM --> MPPI
  SCAN --> COST --> MPPI
  MPPI -->|/cmd_vel_nav| SMOOTH -->|/cmd_vel| WALK
  WALK -->|enable_motion=true only| SPORTAPI
  SPORTAPI -.robot feedback.-> SPORT
```

当前 B2 主 launch 没有启动 Collision Monitor，因此真正的最后软件边界是 `b2_walk`。参数文件中的 Collision Monitor polygon 也处于 disabled；架构图没有把它画成已生效模块。

### B2 主链与早期测试链不要混用

```mermaid
flowchart TB
  subgraph Main[当前 Nav2 主链]
    GOAL[Nav2 goal] --> NAVFN[Navfn]
    NAVFN --> GPU[GPU-MPPI]
    GPU --> VS[Velocity Smoother]
    VS --> BW[b2_walk]
  end

  subgraph Legacy[独立测试/早期链]
    GEN[b2_path_generator] --> PLAN[/plan/]
    PLAN --> PP[b2_path_follower]
    DET[b2_obstacle_detector] --> PP
    PP --> BCMD[/b2/b2cmd_vel/]
  end

  BCMD -.仅显式配置时.-> BW
```

`b2_path_follower` 不是 Nav2 Controller Server 中的 `FollowPath` 插件；它是独立 Pure Pursuit 测试节点。作品集应把这条历史链写成辅助工具，而不是和 GPU-MPPI 主链同时宣传。

## 4. GPU-MPPI 控制周期

```mermaid
flowchart LR
  INPUT[pose + velocity<br/>global path + costmap] --> SAMPLE[采样 N 条控制序列]
  SAMPLE --> ROLLOUT[CUDA 并行 rollout<br/>N × H]
  ROLLOUT --> COST[障碍/路径/航向/<br/>速度/控制变化代价]
  COST --> WEIGHT[MPPI 指数权重]
  WEIGHT --> UPDATE[更新控制序列]
  UPDATE --> FIRST[只执行第 1 步]
  FIRST --> POST[限幅/平滑/安全出口]
  POST --> FEEDBACK[机器人状态反馈]
  FEEDBACK --> INPUT
```

### pluginlib 装载链

```mermaid
flowchart LR
  YAML[FollowPath.plugin] --> RESOURCE[ament resource index]
  RESOURCE --> XML[plugins.xml]
  XML --> LIB[shared library]
  LIB --> EXPORT[PLUGINLIB_EXPORT_CLASS]
  EXPORT --> COMPUTE[computeVelocityCommands]
```

这条链描述控制器从配置到运行时实现的连接关系；控制器性能和安全性需要结合对应的仿真或真机证据判断。

## 5. 模块间稳定接口

| 层 | G1 契约 | B2 契约 | 替换模块时应保持 |
|---|---|---|---|
| 状态 | `/lf/lowstate`, `/odommodestate` | `lf/sportmodestate` | 时间戳、坐标系和消息语义 |
| 雷达 | `/scan` | `/converted_scan` | `sensor_msgs/LaserScan` 与正确 frame |
| 里程计 | `/odom`, `odom -> base_footprint` | `/odom`, `odom -> base` | `nav_msgs/Odometry` 与唯一 TF 发布者 |
| 全局定位 | `map -> odom` | `map -> odom` | 同一时刻只允许一个发布者 |
| 控制器输出 | `/cmd_vel_nav` | `/cmd_vel_nav` | Nav2 controller remap |
| 平滑后 | `/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` |
| 安全后 | `/cmd_vel_safe` | 当前无独立安全话题 | 明确最后执行边界和失效策略 |
| 机器人执行 | API 7105 | SportClient `Move/StopMove` | 默认关闭、有限值、限幅、超时和人工急停 |

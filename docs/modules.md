# 模块说明

## 核心导航包

| 包 | 关键实现 | 设计边界 |
|---|---|---|
| `g1_nav_bringup` | 真机预 SLAM 一键组合：模型、关节、里程计、点云/激光 | 只建立只读数据链，不启动运动桥 |
| `g1_nav_description` | G1 URDF/Xacro、mesh、RViz | 不包含低层关节控制 |
| `g1_nav_state` | `LowState -> JointState` | 只读状态适配 |
| `g1_nav_odometry` | SportModeState 适配、平面坐标转换、TF 发布 | 里程计不等于全局定位 |
| `g1_nav_sensors` | 点云时间戳适配、PointCloud2-to-LaserScan、speckle filter | 输出 Nav2 使用的二维扫描 |
| `g1_nav_slam` | mapping/localization launch、保存脚本 | 真实地图不随公开仓库分发 |
| `g1_nav_nav2` | Navfn、RPP/MPPI、BT、costmap、smoother、Collision Monitor | 输出停在 `/cmd_vel_safe` |
| `g1_nav_control` | API 7105 JSON 请求、watchdog、限幅、无运动门控 | 不实现步态和关节控制 |
| `g1_nav_sim` | Gazebo world、运动代理、Mid360、地图、RViz | 是二维导航集成测试，不是双足动力学仿真 |

## 控制器包

### `nav2_custom_plugins`

- `MPCController`：确定性模型预测控制基线。
- `MPPIController`：CPU 采样式控制器。
- `MPPIGPUController`：CUDA 批量 rollout 与代价评估。
- `EscapeObstacle`：恢复行为原型。
- 自适应进度检测源码仍是未编译原型，未在 `plugins.xml` 中对外声明，避免误用。

### `nav2_custom_plugins_v2`

v2 将单体控制器拆为：

```text
Nav2 adapter
  -> parameter loader
  -> path manager
  -> MPPI core/pipeline
  -> GPU engine + uploader + critics
  -> state machine
  -> velocity postprocessor
  -> visualization
```

该分支适合研究 critic 组合、控制空间、动力学模型和 GPU 性能；它不是默认真机控制器。

## 外部接口

| 接口 | 类型/语义 |
|---|---|
| `/lf/lowstate` | Unitree G1 低层只读状态 |
| `/odommodestate` | Unitree 高层运动状态，用作本地里程计输入 |
| `/utlidar/cloud_livox_mid360` | 真机三维点云 |
| `/scan` | 过滤后的二维障碍扫描 |
| `/odom` | Nav2 标准里程计 |
| `/cmd_vel_safe` | Collision Monitor 之后的唯一执行输入 |
| `/api/sport/request` | Unitree 高层 API 请求；默认不发布 |
| `/cmd_vel_g1` | 最终限幅后的诊断速度，不会自行启用运动 |

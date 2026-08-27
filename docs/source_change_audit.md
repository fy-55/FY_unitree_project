# 发布包装源码变更审计

本文件记录为了公开发布、安全默认值和跨机器复现所做的源码/配置调整。两个原真机工作区均保留了修改前的同目录 `.bak`，备份不进入公开仓库。

## 没有修改的核心

- G1/B2 的 MPPI、GPU kernel、MPC、代价函数和控制器主循环。
- G1/B2 的 Nav2 主参数（B2 的定位地图占位参数除外）。
- G1 的 API 7105 速度桥 C++ 实现、里程计、关节状态和点云适配实现。
- B2 的状态发布、雷达 relay、SportClient 实现及实验计算逻辑。
- 真实地图、rosbag、构建目录和设备数据。

## G1 行为相关调整

| 文件 | 调整 | 真机影响 |
|---|---|---|
| `g1_nav_control/config/velocity_bridge.yaml` | 默认最大前向速度 `0.90 → 0.10 m/s`，角速度 `1.50 → 0.20 rad/s` | 会降低首次真机速度；不改变话题/API 链 |
| `g1_nav_slam/launch/localization.launch.py` | 默认地图目录改为 package share | 去除本机绝对路径；显式 `map_dir` 仍可覆盖 |
| `g1_nav_slam/save_map.sh` | 默认保存到用户数据目录，可通过环境变量覆盖 | 地图不再默认写入源码目录 |
| `g1_nav_sim/env/use_g1_sim.sh` | 从脚本位置推导 workspace | 只影响仿真环境加载，不影响真机 DDS |
| `g1_nav_slam/CMakeLists.txt` | 安装公开地图目录 | 让安装空间能够找到仿真/示例地图 |

此外只调整了 package 元数据、许可证、缺失依赖和插件注册表。`AdaptiveProgressChecker` 源码仍保留，但因为它没有进入构建目标且接口未验证，所以不再宣称为可加载插件；默认配置原本就使用 `SimpleProgressChecker`。

## B2 行为相关调整

| 文件 | 调整 | 真机影响 |
|---|---|---|
| `b2_driver/src/b2_walk.cpp` | 新增 `enable_motion=false`、NaN/Inf 拒绝、速度限幅、参数校验 | 单独运行节点时默认不向机器人发运动 API；真机需显式 `-p enable_motion:=true` |
| `scripts/start_b2_navigation.sh` | 默认不启动运动桥；要求显式序列化地图；`--walk` 才启用运动 | 原来的无参数自动行走行为被改为无运动预检 |
| `b2_navigation/launch/b2_localization.launch.py` | 新增显式 `map_file_name` 的定位入口 | 使用自己的 `.posegraph/.data` 地图基名启动定位 |
| `b2_navigation/params/slam_toolbox_localization.yaml` | 移除本机硬编码地图路径 | 不再误用开发机地图；地图由启动参数提供 |

实验环境脚本只将本机绝对路径改为从脚本位置推导。三个 Python launch 文件仅清理未使用 import、变量命名和空白，以通过 lint；节点、话题和参数未改变。

package/CMake 调整只补充实际使用的依赖、许可证和自动检查范围。`AdaptiveProgressChecker` 与 G1 相同：源码保留但不再注册为已构建插件。

## 继续真机运行

G1 仍使用原来的启动链，但会采用更低的公开默认限速。B2 使用：

```bash
./scripts/start_b2_navigation.sh --map-base=/absolute/path/to/map
```

先检查定位、路径和 `/cmd_vel`。完成现场安全检查后，才追加：

```bash
./scripts/start_b2_navigation.sh \
  --map-base=/absolute/path/to/map \
  --walk --log
```

如需完全恢复包装前行为，应在原真机工作区逐文件对照同目录 `.bak`，不要直接覆盖整个 workspace；恢复后必须重新构建，并重新检查地图路径、速度值和运动出口。

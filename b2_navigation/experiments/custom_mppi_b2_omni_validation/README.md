# 自定义 GPU-MPPI 的 B2式全向二维隔离验证

这个实验把控制闭环改成：

```text
FollowPath
  -> /cmd_vel_nav
  -> 当前项目同款 velocity_smoother
  -> /cmd_vel
  -> 支持 vx、vy、wz 的 Gazebo planar plant
  -> /odom + TF + /scan
  -> 下一轮 FollowPath
```

假底盘外形为 `1.0 m × 0.6 m`，与当前自定义控制器使用的前后半长
`0.5 m`、左右半宽 `0.3 m` 对齐。它是理想速度响应的二维对象，不是腿式
动力学，也不代表真实B2。

Nav2的进度检查和到点容差保留当前B2项目设置：
`required_movement_radius=0.5 m`、`xy_goal_tolerance=0.35 m`、
`yaw_goal_tolerance=0.60 rad`。

## 安全边界

- 不修改或重新编译 `src/b2_navigation`；
- 不修改或重新编译 `src/nav2_custom_plugins`；
- 不启动任何Unitree/B2节点；
- 使用独立 `ROS_DOMAIN_ID=44`、本机通信和Gazebo端口 `11347`；
- 所有配置、日志和结果保存在本目录。

## 启动

```bash
cd <repository>/experiments/custom_mppi_b2_omni_validation
python3 prepare_configs.py
bash scripts/check_isolation.sh
bash scripts/start_sim.sh custom
```

另一个终端运行纯横移目标：

```bash
bash scripts/run_trial.sh \
  --variant custom \
  --scenario lateral \
  --trial smoke_01 \
  --goal-x -2.0 \
  --goal-y 0.3
```

官方全向MPPI参考组使用 `bash scripts/start_sim.sh official`，每组试验前都要
重启Gazebo，保证起点完全相同。

目标 `(-2.0, 0.3)` 相对起点是约 `0.8 m` 的纯横移，并且机器人矩形足迹
可以安全到达。不要使用 `(-2.0, 1.0)` 作为正式对照；它离障碍中心线仅约
`0.283 m`，小于假B2的 `0.30 m` 半宽，规划器会把路径终点移到别处。

## 当前结果

已经完成同一可达纯横移目标的自定义/官方正式冒烟对照。重新生成汇总：

```bash
python3 analyze_results.py
```

产物：

```text
results/formal_summary.csv
results/omni_validation_overview.png
VALIDATION_REPORT.md
```

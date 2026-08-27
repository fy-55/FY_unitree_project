# 自定义 GPU-MPPI 的隔离 Nav2/Gazebo 验证

这个目录验证当前已编译的
`nav2_custom_plugins/MPPIGPUController` 是否能够作为 Nav2 `FollowPath`
控制器完成二维导航，并与官方 Nav2 MPPI 做同场景参考。

## 安全边界

- 不修改 `src/b2_navigation` 或 `src/nav2_custom_plugins`；
- 不构建工作区，不启动 `b2_walk`、B2传感器或Unitree SDK节点；
- 使用 `ROS_DOMAIN_ID=43` 和 `ROS_LOCALHOST_ONLY=1`；
- 使用独立 Gazebo 端口 `11346`；
- 所有新配置、ROS日志和结果都保存在本实验目录。

这里使用的是差速 TurtleBot3 Waffle。自定义控制器的最大速度、横向速度和
碰撞箱按 TurtleBot3 做兼容性适配，但核心仍保留当前的 `8000` 条候选、
`H=5`、`dt=0.1s`、混合采样、warm-start、代价比例和GPU rollout。
因此它可以验证插件加载、二维闭环、基础避障和控制输出，不能代替全向B2
仿真，更不能作为真实B2性能结论。

## 生成隔离配置

```bash
cd <repository>/experiments/custom_mppi_nav2_validation
python3 prepare_configs.py
bash scripts/check_isolation.sh
```

## 启动自定义控制器

```bash
bash scripts/start_sim.sh custom
```

另一个终端使用相同环境并发送固定目标：

```bash
bash scripts/run_trial.sh --variant custom --trial smoke_01
```

官方参考组：

```bash
bash scripts/start_sim.sh official
bash scripts/run_trial.sh --variant official --trial smoke_01
```

每次比较必须重启Gazebo，使机器人从同一位置开始。第一阶段只做冒烟试验；
两个控制器都能稳定完成后，再运行重复试验和增加墙角、窄通道目标。

## 当前初步结果

已经完成一组直线自定义控制器试验，以及同一转弯目标的自定义/官方对照。
生成汇总图和CSV：

```bash
python3 analyze_results.py
```

产物：

```text
results/
├── trial_summary.csv
├── validation_overview.png
└── trials/
```

完整结果、日志证据和结论边界见 `VALIDATION_REPORT.md`。

# B2 GPU-MPPI 运动模型失配：离线敏感性实验

## 2026-07-28 更新：完整一维实验

原来的 `run_experiment.py` 和 `run_closed_loop_demo.py` 是组会前的最小敏感性演示，使用了手工简化代价、单一随机种子，并且没有显式复现当前 Nav2 速度平滑器。它们保留用于追溯，但不再作为当前主要证据。

新的完整一维审计入口：

```bash
cd <repository>
python3 experiments/mppi_model_mismatch/run_full_1d_experiment.py
```

如果正式 CSV 已经存在，只重新生成“MPPI 原始指令 → 外部平滑器输出 →
人工对象实际速度 → 位置反馈”桥梁图，不重跑 Monte Carlo：

```bash
python3 experiments/mppi_model_mismatch/run_full_1d_experiment.py \
  --execution-plot-only
```

它增加了：

- 从当前 `dog_nav_params.yaml` 直接加载有效参数；
- 当前 GPU-MPPI 纵向采样、warm-start、代价和加权逻辑的一维投影；
- Nav2 Humble `OPEN_LOOP` velocity_smoother 的源代码等价实现；
- 理想、平滑感知、完整响应感知三组消融；
- 同一批 8000 条候选的直接排序重算；
- 20 个随机种子和延迟/时间常数网格；
- 当前 `0.35m` 目标检查容差；
- 三种速度和同周期位移的位置反馈桥梁图；
- 参数与关键源代码哈希快照。

完整方法、结果、边界和结论见 `FULL_1D_REPORT.md`。

当前正式结论是：人工压力条件下候选排序明显改变，但合法墙前目标的所有试验仍保持安全；旧版单随机种子“越过安全边界 7.7mm”的结论已被取代，不应继续作为当前实验结论。

这是为 2026-07-28 组会准备的一个可复现小实验。目标不是声称已经得到真机结论，而是把代码审计中发现的模型假设转化为一个可以验证的研究问题。

## 一句话结论

当前 GPU-MPPI 使用候选 `vx/vy/omega` 直接积分未来位姿。如果 B2 对速度命令存在延迟、滞后或比例误差，预测的候选轨迹代价可能与真实执行结果不一致。离线实验说明了这种失配“可能造成什么影响”，真实影响大小仍需用 B2 的 `/cmd_vel` 与 `/odom` 数据标定。

## 来自现有工程的代码事实

- `controller_frequency: 20.0`，控制器周期为 `0.05s`。
- `num_samples: 8000`、`prediction_horizon: 5`、`dt: 0.1`，单次预测时域为 `0.5s`。
- GPU 内核通过 RK2 将候选速度直接积分为未来位姿。
- `computeVelocityCommands()` 当前未使用传入的 `current_vel`。
- `velocity_smoother` 为 `OPEN_LOOP`，并配置了速度、加速度和死区限制。

对应位置：

- `src/b2_navigation/params/dog_nav_params.yaml:59`
- `src/b2_navigation/params/dog_nav_params.yaml:201`
- `src/b2_navigation/params/dog_nav_params.yaml:445`
- `src/nav2_custom_plugins/src/mppi_gpu_controller.cpp:414`
- `src/nav2_custom_plugins/src/mppi_gpu_kernels.cu:85`

## 实验模型

给定相同的横向速度命令，比较三种假设执行模型：

| 模型 | 延迟 | 一阶时间常数 | 稳态速度比例 | 用途 |
|---|---:|---:|---:|---|
| 理想模型 | 0s | 0s | 1.00 | 对应当前直接积分假设 |
| 轻度失配 | 0.05s | 0.15s | 0.85 | 敏感性测试，不代表真实 B2 |
| 严重失配 | 0.10s | 0.30s | 0.50 | 压力测试，不代表真实 B2 |

非理想模型使用：

```text
dv/dt = (scale * u(t-delay) - v) / tau
```

其中参数是人为设置的测试条件，不是真机辨识结果。

## 运行

```bash
cd <repository>
python3 experiments/mppi_model_mismatch/run_experiment.py
```

输出位于：

```text
experiments/mppi_model_mismatch/results/
├── velocity_response.png
├── horizon_displacement.png
├── clearance_classification.png
├── timing_chain.png
├── summary.csv
└── candidate_clearance.csv
```

## 现场闭环演示

开环曲线只说明“同一命令可能产生不同响应”。下面的独立脚本进一步形成一个
最小闭环：每个控制周期实际采样 8000 条长度为 5 的控制序列，按 MPPI
指数权重更新控制指令，再把第一条指令交给同一个带延迟、惯性的虚拟对象。

```bash
cd <repository>
python3 experiments/mppi_model_mismatch/run_closed_loop_demo.py
```

它比较两种控制器：

1. 理想模型 MPPI：候选速度直接积分成位移；
2. 响应感知 MPPI：预测中加入与虚拟对象一致的延迟和一阶响应。

新增输出：

```text
experiments/mppi_model_mismatch/results/
├── closed_loop_demo.gif
├── closed_loop_comparison.png
├── closed_loop_summary.csv
└── closed_loop_timeseries.csv
```

演示沿用当前工程中的 `20Hz`、`8000` 候选、`H=5`、`dt=0.1s` 和
`temperature=4.0`，但代价函数是为了说明因果关系而编写的最小版本，
并不是 Nav2 全部 critic 的复刻。对象的 `0.10s` 延迟和 `τ=0.45s`
是人为压力测试条件，不代表 B2 实测数据。这个脚本可以称为
“独立闭环原型”或“压力测试”，不能称为“B2 仿真结果”或“Nav2 完整复现”。

## 当前结果应该怎样表述

可以说：

> 我从现有 GPU-MPPI 实现中提取了理想运动学假设，并做了命令延迟、速度滞后和比例误差的离线敏感性分析。结果表明，在人为设置的失配条件下，预测净空判断可能发生错误，因此下一步有必要通过仿真闭环和真机低速数据检验这一问题。

不能说：

- “已经证明 B2 存在这些具体误差”；
- “这些曲线是真机数据”；
- “已经提出并验证了创新算法”；
- “已经证明能够提升真实机器人避障性能”。

## 下一步

1. 建立不连接真机的二维假 B2 闭环，发布 `/odom`、TF 和假激光。
2. 让现有 Nav2 + GPU-MPPI 在理想、延迟和打滑模型下完成相同路线。
3. 记录 `/cmd_vel_nav`、`/cmd_vel` 和 `/odom`，比较成功率、路径误差、最小障碍距离和控制抖动。
4. 最后进行低速真机测试，用真实数据标定延迟、时间常数和速度比例。

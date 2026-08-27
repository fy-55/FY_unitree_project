# 控制器设计

## 可用实现

| 控制器 | 实现 | 定位 |
|---|---|---|
| MPC | `mpc_controller.cpp` | 确定性基线 |
| CPU-MPPI | `mppi_controller.cpp` | 便于理解和调试的采样基线 |
| GPU-MPPI | `mppi_gpu_controller.cpp` + CUDA kernels | 当前 B2 Nav2 配置 |

## GPU-MPPI 一次控制周期

1. 接收 Nav2 全局路径并转换到控制坐标系。
2. 从当前均值控制序列采样大量候选 `[vx, vy, wz]`。
3. CUDA kernel 并行积分候选轨迹。
4. 从 costmap、路径与控制序列计算各项代价。
5. 按 MPPI 权重更新控制分布。
6. 对首条命令做后处理并返回 Nav2。
7. 发布调试轨迹/统计，供时限与行为分析。

配置基线：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `controller_frequency` | 20 Hz | Nav2 控制周期 |
| `num_samples` | 8000 | 每周期候选数 |
| `prediction_horizon` | 5 | rollout 步数 |
| `dt` | 0.1 s | 模型积分步长 |
| `velocity_smoother.feedback` | OPEN_LOOP | 平滑器不使用实测速度闭环 |

## 代价与扩展

代码将路径跟踪、障碍、航向、速度、控制变化、安全走廊等因素组合成候选总代价。新增 critic 时需要同时定义：

- 输入数据和坐标系；
- 权重与量纲；
- GPU kernel 中的计算；
- 无效/越界处理；
- 至少一个可失败的单元或离线测试。

## 已知研究问题

当前 rollout 直接积分采样速度，并未证明等于真实 B2 的执行响应。若机器人存在延迟、滞后、死区或打滑，候选轨迹排序和碰撞代价可能改变。因此项目先保留理想模型基线和模型失配审计，再决定是否增加一阶响应、在线辨识或残差模型。

自适应进度检测器仍是未编译原型，已从 `plugins.xml` 移除，不能当作当前可加载功能。

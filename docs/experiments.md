# 实验与证据

## 实验目录

| 目录 | 目的 | 可得结论 |
|---|---|---|
| `mppi_model_mismatch` | 延迟、滞后、死区等假设下的一维闭环与候选排序审计 | 模型失配可能影响排序；不能反推真实 B2 参数 |
| `nav2_official_mppi_sim` | 官方 Nav2 MPPI/TurtleBot3 基线 | 验证隔离 Nav2 流程 |
| `custom_mppi_nav2_validation` | 自定义 GPU-MPPI 在同类 Nav2 环境中的运行 | 控制器能否装载、运行和到达目标 |
| `custom_mppi_b2_omni_validation` | 更接近 B2 全向接口的代理场景 | 检查 `vx/vy/wz` 与模型失配敏感性 |
| `b2_execution_risk_mppi_paper` | 论文实验计划、假设和继续/停止门 | 是研究计划，不是论文结论 |

## 当前可重复的最小证据

```bash
python3 experiments/mppi_model_mismatch/run_full_1d_experiment.py --self-test-only
```

当前历史审计中该命令通过。完整一维报告明确区分人为失配参数与真实 B2 测量，见 `FULL_1D_REPORT.md`。

## 不能混为一谈

- YAML 参数不是机器人动力学测量。
- GPU library 构建成功不是 Nav2 目标成功。
- 单次仿真成功不是统计性能提升。
- TurtleBot3/全向代理不是 B2 真机。
- “成功后约移动几厘米”等观察不是正式轨迹数据。
- 计划图、预期结果和论文假设不是实验结果。

## 下一步证据门

1. 记录 `/cmd_vel_nav`、`/cmd_vel`、B2 SportModeState 的源时间与接收时间。
2. 用外部定位或独立参考检查执行速度、延迟和打滑。
3. 先验证执行误差是否足以改变候选排序。
4. 若影响小，停止复杂模型方向；若影响稳定且显著，再比较一阶模型、在线辨识或残差模型。
5. 在统一场景、路线、随机种子和安全阈值下报告成功率、净空、误差、耗时、控制抖动和周期超期率。

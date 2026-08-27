# 验证状态与证据边界

本项目用不同词描述不同强度的证据，避免把“代码存在”写成“机器人已验证”。

| 层级 | 能证明什么 | 当前公开状态 |
|---|---|---|
| Source-present | 模块、参数和启动文件存在 | 已具备 |
| Static checks | Python/Shell/XML/YAML 可解析，安全默认值存在 | `scripts/check_source.sh` |
| Build | 当前机器上的依赖和 ABI 能编译链接 | 2026-08-27：13 个相关包构建通过 |
| Offline interface | 话题、TF、lifecycle 与无运动输出符合预期 | 有明确验收命令 |
| Gazebo integration | 仿真传感器、定位、Nav2、速度链可闭环 | 提供可复现流程；需保留本次运行日志作为证据 |
| Physical no-motion | 真机输入可读，API 输出保持关闭 | 必须在目标机器重新验证 |
| Physical motion | 真实 G1 受控到达目标并满足安全指标 | 公开仓库当前不作此结论 |

## 本次发布检查

- 环境：Ubuntu 22.04、ROS 2 Humble、GCC 11.4、RTX 4060 Laptop GPU、CUDA 13.2。
- `colcon build`：13 个相关包完成，`nav2_custom_plugins` 两个分支存在编译器警告但无错误。
- `colcon test`：110 项结果，0 error、0 failure、6 skipped；当前 cppcheck 2.7 因 ament 的已知性能限制被跳过。
- `scripts/check_source.sh`：通过。

这些结果只证明当前主机上的源码、链接和静态质量门，不替代 Gazebo 闭环或真机实验。

## 已知限制

- Gazebo 使用二维运动代理，不代表 G1 双足动力学或真实刹停性能。
- `SportModeState` 里程计不等于外部高精度真值。
- Collision Monitor 参数尚需用真实刹停距离标定。
- GPU-MPPI 的构建和离线运行不等于它优于 RPP，也不等于已在 G1 真机验证。
- 当前包主要依赖 lint/静态检查，尚缺系统化单元测试、bag 回放回归与多场景统计。
- 自适应进度检测器源码是未编译原型，未注册为可加载插件。

## 建议补充到作品集的证据

1. 30–60 秒无剪辑仿真视频：地图、激光、全局路径、局部轨迹和速度链同时可见。
2. 每次测试保存版本号、配置、目标点、成功/失败、最小净空与耗时。
3. 真机阶段先上传无运动话题/TF 检查，再上传低速单目标；不要用宣传性文字替代数据。
4. 将 rosbag 和完整地图放在私有存储，只在仓库发布匿名化统计与示意图。

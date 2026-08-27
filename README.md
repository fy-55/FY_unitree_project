# Unitree G1 & B2 Robotics Portfolio

这是一个面向 Unitree G1 与 B2 机器人平台的工程与研究项目集合，包含两个自主导航项目和一个 G1 语音交互项目。三个项目彼此独立，分别拥有自己的运行环境、依赖和入口。

## 项目总览

| 项目 | 平台 | 核心能力 | 主要技术 |
|---|---|---|---|
| [G1 Navigation](g1_navigation/README.md) | Unitree G1 | 室内建图、定位、路径规划和安全运动控制 | ROS 2、SLAM Toolbox、Nav2、Gazebo |
| [B2 Navigation](b2_navigation/README.md) | Unitree B2 | 全流程导航、GPU-MPPI 控制和模型失配实验 | ROS 2、CUDA、Nav2、MPPI |
| [G1 Voice Interaction](g1_voice_interaction/README.md) | Unitree G1 | ASR 语音输入、自然语言对话、TTS 播报和有限机器人动作 | C++17、Unitree SDK2、Ollama、G1 ASR/TTS |

## G1 Navigation

```text
G1 状态 / Mid360 点云
        -> ROS 2 数据适配
        -> 里程计、TF 与 LaserScan
        -> SLAM Toolbox / 地图定位
        -> Nav2 规划与控制
        -> Collision Monitor
        -> 速度安全门与 Unitree API 7105
```

G1 导航项目覆盖机器人状态、点云和里程计适配，SLAM 建图与定位，Nav2 全局规划和局部控制，以及连接 Unitree 高层运动接口的安全速度桥。项目提供 Gazebo 导航代理、虚拟 Mid360、示例地图和离线检查工具。

项目入口：[g1_navigation/README.md](g1_navigation/README.md)

## B2 Navigation

```text
B2 SportModeState / RoboSense 点云
        -> 状态与雷达适配
        -> 里程计、TF 与 LaserScan
        -> SLAM Toolbox / 地图定位
        -> Nav2 + GPU-MPPI
        -> 速度平滑
        -> b2_walk 安全运动出口
```

B2 导航项目重点研究 CPU/GPU MPPI、CUDA 并行 rollout、控制代价计算、Nav2 pluginlib 接入、恢复行为和模型失配实验。公开内容同时区分标准导航链、离线实验和真实机器人运行边界。

项目入口：[b2_navigation/README.md](b2_navigation/README.md)

## G1 Voice Interaction

```text
G1 ASR
  -> 文本清理与唤醒
  -> 有界消息队列
  -> 命令路由
       |-> 本地 Ollama 对话 -> G1 TTS
       |-> 手臂动作白名单
       |-> 限速有限时长前进
       `-> StopMove 与复位
```

G1 语音项目实现从语音识别到语音播报的完整人机交互闭环。普通对话使用本地 Ollama/Qwen 模型；机器人动作由本地白名单控制，不执行模型生成的任意机器人函数；运动功能默认关闭。

项目入口：[g1_voice_interaction/README.md](g1_voice_interaction/README.md)

## 项目结构

```text
unitree_navigation_portfolio/
├── g1_navigation/        # G1 ROS 2 导航项目
├── b2_navigation/        # B2 ROS 2 导航项目
├── g1_voice_interaction/ # G1 C++ 语音交互项目
└── docs/                 # 组合项目的架构与范围说明
```

## 文档导航

- [系统架构](docs/system_architecture.md)：三个项目的总体数据流、模块关系和接口边界。
- [功能包与模块目录](docs/module_catalog.md)：G1、B2 导航功能包及实验模块说明。
- [项目组织说明](docs/development_workflow.md)：三个项目的组成、运行环境和功能关系。
- [项目范围与安全边界](docs/source_change_audit.md)：运动控制、实验结论和公开内容边界。

## 复现入口

G1 和 B2 导航项目分别按照各自 README 与 `docs/reproduction.md` 中的环境要求运行。语音项目可在其目录中进行纯逻辑测试：

```bash
cd g1_voice_interaction
cmake -S . -B build -DUNITREE_SDK_ROOT=/path/to/unitree_sdk2
cmake --build build --target g1_audio_client_test g1_voice_logic_test -j4
./build/g1_voice_logic_test
```

语音项目的本地模型运行需要 Ollama、Qwen 模型和 G1 网络连接；真机运动必须保留遥控器或硬件急停。

## 安全边界

- G1 导航默认不向机器人发送运动指令。
- B2 `b2_walk` 默认关闭物理运动，实验程序不作为常规导航入口。
- G1 语音项目默认关闭行走，只允许本地固定动作白名单。
- 公开仓库不包含 API Key、私有模型、真实场地图、rosbag、设备网络配置或构建产物。
- 代码构建、离线测试、仿真和真机测试代表不同层级的证据，不能相互替代。
- 所有真机运动都需要现场安全检查和独立的硬件急停措施。

## 许可证

项目中的原创部分采用 [Apache-2.0](LICENSE)。Unitree SDK、机器人模型、消息接口、ROS 2 组件和其他第三方依赖沿用各自许可证，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 及各子项目说明。

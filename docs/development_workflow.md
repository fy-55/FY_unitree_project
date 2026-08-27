# 项目组织说明

本作品集由三个相互独立的项目组成：

| 项目 | 内容 |
|---|---|
| G1 Navigation | 基于 ROS 2、SLAM Toolbox 和 Nav2 的 G1 导航系统 |
| B2 Navigation | 基于 ROS 2、GPU-MPPI 和安全运动桥的 B2 导航系统 |
| G1 Voice Interaction | 基于 ASR、LLM 和 TTS 的 G1 语音交互系统 |

## 项目边界

- G1 Navigation 和 B2 Navigation 是独立的 ROS 2 workspace。
- G1 Voice Interaction 是独立的 C++ 语音应用项目，不依赖导航 workspace 的 ROS 包。
- 三个项目分别拥有自己的依赖、参数、启动入口和运行环境。
- 本仓库不包含真实场地图、rosbag、本机构建目录、设备配置或私有模型文件。

## 功能关系

导航项目关注机器人感知、定位、路径规划和运动控制；语音项目关注语音输入、自然语言理解、语音输出以及有限的机器人动作控制。语音项目可以作为独立的人机交互入口，不改变导航系统的内部模块边界。

## 运行环境

| 项目 | 主要运行环境 |
|---|---|
| G1 Navigation | ROS 2、Gazebo、SLAM Toolbox、Nav2 |
| B2 Navigation | ROS 2、CUDA、SLAM Toolbox、Nav2 |
| G1 Voice Interaction | C++17、Unitree SDK2、libcurl、本地 Ollama、G1 ASR/TTS |

详细功能和运行入口分别见各项目 README 及对应复现说明。

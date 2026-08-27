# Unitree G1 & B2 二次开发项目介绍

## 中文版本

本项目是针对宇树科技（Unitree）G1 与 B2 两款机器人平台的二次开发成果集合。

### 自主导航

项目为 G1 和 B2 构建了完整的自主导航应用闭环。技术链路涵盖：

- 订阅宇树官方数据源并转换为 ROS2 标准消息格式；
- 调用 `slam_toolbox` 实现即时建图与重定位；
- 集成 Nav2 导航栈完成路径规划与运动控制；
- 自主开发了速度桥接节点以适配底层驱动。

该链路层层递进、结构清晰，非常适合刚接触 ROS2 机器人导航的初学者从零起步学习。

### 语音交互

项目为 G1 单独开发了专属语音模块，实现了从 ASR（自动语音识别）前端唤醒，到 TTS（语音合成）播报输出的全流程人机语音交互闭环。

---

## English Version

This project is a comprehensive secondary development suite for Unitree's G1 and B2 robotic platforms.

### Autonomous Navigation

We have established a complete closed-loop navigation application for both the G1 and B2. The technical pipeline involves:

- Subscribing to Unitree's official data streams and converting them into ROS2 standard message types;
- Leveraging `slam_toolbox` for real-time mapping and relocalization;
- Integrating the Nav2 stack for path planning and motion control;
- Implementing a custom velocity bridging node to interface with low-level drivers.

With its well-structured, step-by-step architecture, this project serves as an excellent entry point for beginners starting from scratch with ROS2 robot navigation.

### Voice Interaction

We have developed a dedicated voice module specifically for the G1, which realizes a complete human-robot interaction closed-loop—from ASR (Automatic Speech Recognition) wake-up to TTS (Text-to-Speech) audio output.

# Unitree G1 Voice Interaction

面向 Unitree G1 的语音交互项目说明与复现索引。

本项目展示一条完整的人机语音闭环：

```text
G1 ASR -> 文本清理与唤醒 -> 有界消息队列 -> 命令路由
                                      |-- 普通问题 -> OpenAI 兼容大模型 -> G1 TTS
                                      |-- 手臂命令 -> 本地动作白名单
                                      |-- 前进命令 -> 有限时长 Move -> StopMove
                                      `-- 停止命令 -> StopMove 与手臂复位
```

## 项目定位

这是作品集中的独立项目。正式 C++ 源码已按正式主线复制到本目录。项目包含可查看和编译的语音源码副本，但不包含 Unitree SDK、模型文件、构建产物或私有配置。

## 文档导航

- [功能介绍](docs/features.md)：交互能力、命令边界和安全策略。
- [系统结构](docs/architecture.md)：正式入口、模块职责和数据流。
- [复现流程](docs/reproduction.md)：依赖、编译、本地模型和真机步骤。
- [源码边界与发布说明](docs/source_boundary.md)：哪些内容属于原项目，哪些内容可以公开提交。

## 快速了解

| 项目 | 说明 |
| --- | --- |
| 输入 | G1 自带 ASR 识别的中文语音 |
| 对话 | 本地 Ollama/Qwen 模型 |
| 输出 | G1 TTS 语音播报 |
| 动作 | 本地白名单动作 ID，不执行模型生成的任意函数 |
| 移动 | 默认关闭；开启后仅允许低速、有限时长前进 |
| 测试 | `g1_voice_logic_test` 只测试文本处理和命令路由，不连接机器人 |

## 复现入口

在新项目目录中，先准备 Unitree SDK2、系统依赖和 G1 语音接口，然后：

```bash
cd g1_voice_interaction
cmake -S . -B build
cmake --build build --target g1_audio_client_test g1_voice_logic_test -j4
./build/bin/g1_voice_logic_test
```

如果 SDK 不在系统默认路径，配置时增加 `-DUNITREE_SDK_ROOT=/path/to/unitree_sdk2`。

本地模型启动和真机运行顺序见 [复现流程](docs/reproduction.md)。真机实验必须保留遥控器或急停，不能把语音停止当作唯一安全措施。

## 许可证与第三方组件

Unitree SDK2、G1 消息接口、模型、音频接口和其他第三方组件沿用各自许可证；公开发布前请同时检查原项目的 `LICENSE` 与 `THIRD_PARTY_NOTICES.md`。

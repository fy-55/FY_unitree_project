# 复现流程

## 0. 复现边界

本目录包含正式语音 C++ 源码副本和构建脚本，但不包含 Unitree SDK、模型文件、构建目录或真实配置。首次复现可在本项目目录执行：

```bash
cd g1_voice_interaction
```

如果 SDK 不在默认位置，配置时使用 `-DUNITREE_SDK_ROOT=/path/to/unitree_sdk2`。

## 1. 安装依赖

Ubuntu 示例：

```bash
sudo apt install cmake g++ make libcurl4-openssl-dev nlohmann-json3-dev
```

还需要与目标架构匹配的 Unitree SDK2 头文件和预编译库，以及可用的 G1 ASR/TTS 接口。

## 2. 纯本机逻辑测试

该测试不初始化机器人，不连接网卡，不发送动作：

```bash
cmake -S . -B build -DUNITREE_SDK_ROOT=/path/to/unitree_sdk2
cmake --build build --target g1_audio_client_test g1_voice_logic_test -j4
./build/bin/g1_voice_logic_test
```

建议先通过这一步确认文本清理、唤醒词、命令分类和白名单行为。

## 3. 本地 Ollama 模式

准备本地模型服务配置：

```bash
cp scripts/g1_voice_local.env.example scripts/g1_voice_local.env
chmod 600 scripts/g1_voice_local.env
```

按实际安装位置编辑 `scripts/g1_voice_local.env`，然后在第一个终端启动 Ollama：

```bash
./scripts/start_ollama.sh
```

保持该终端运行。在第二个终端确认网卡名：

```bash
ip -br link
```

启动语音助手：

```bash
./scripts/run_g1_voice.sh <G1有线网卡名>
```

第一次对话可能触发模型加载。纯文字接口可先用 `./scripts/test_local_llm.sh` 检查模型服务。

## 4. 真机前检查

1. 确认 G1 电量、网络和 ASR/TTS 服务正常。
2. 确认站立状态和底层 FSM 满足动作调用条件。
3. 前方留出空旷区域，旁边安排一名操作人员。
4. 保留遥控器或硬件急停，并先保持 `G1_ENABLE_WALKING=0`。
5. 先测试普通问答，再测试固定手臂动作，最后才在受控环境启用行走。
6. 退出程序后确认机器人已停止运动。

“向前走两步”是速度乘时间的近似控制，不是基于里程或脚步的闭环控制。任何真机试验都应按现场安全规程执行。

## 5. 常用配置

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `G1_WAKE_WORD` | `小贝同学` | 唤醒词 |
| `G1_WAKE_WINDOW_MS` | `8000` | 分两句说话时的等待窗口 |
| `G1_VOLUME` | `60` | 音量，范围 0 至 100 |
| `G1_ENABLE_WALKING` | `0` | 是否允许行走 |
| `G1_WALK_SPEED` | `0.12` | 前进速度，范围 0.05 至 0.30 m/s |
| `G1_WALK_DURATION_MS` | `1500` | 前进持续时间 |
| `LLM_MAX_TOKENS` | `512` | 模型输出上限 |
| `LLM_REQUEST_TIMEOUT_SECONDS` | `120` | 请求总超时 |

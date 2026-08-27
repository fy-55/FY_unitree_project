#include "voice_chat/asr_processor.hpp"
#include "voice_chat/command_router.hpp"

#include <chrono>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void Check(bool condition, const std::string &name) {
  if (condition) return;
  ++g_failures;
  std::cerr << "测试失败：" << name << std::endl;
}

}  // namespace

int main() {
  using namespace std::chrono_literals;
  using g1_voice::CommandType;

  const auto json_text = g1_voice::ExtractAsrText(
      R"({"text":"向前走两步","is_final":false})");
  Check(json_text && *json_text == "向前走两步", "解析G1的JSON ASR消息");

  const auto plain_text = g1_voice::ExtractAsrText("  你好  ");
  Check(plain_text && *plain_text == "你好", "兼容纯文本ASR消息");

  const auto no_text = g1_voice::ExtractAsrText(R"({"status":"playing"})");
  Check(!no_text, "忽略不含text的音频状态消息");

  g1_voice::WakeWordGate gate("小贝同学", 5s);
  const auto same_sentence = gate.Accept("小贝同学，向前走两步");
  Check(same_sentence && *same_sentence == "向前走两步",
        "同一句话中的唤醒词和命令");

  const auto wake_only = gate.Accept("小贝同学");
  Check(!wake_only && gate.WaitingForCommand(), "单独唤醒后打开命令窗口");
  const auto next_sentence = gate.Accept("介绍一下你自己");
  Check(next_sentence && *next_sentence == "介绍一下你自己",
        "唤醒窗口接收下一句话");

  Check(g1_voice::RouteCommand("请和我握手").type == CommandType::kArmAction,
        "握手命令路由");
  Check(g1_voice::RouteCommand("往前走两步").type ==
            CommandType::kForwardTwoSteps,
        "前进命令路由");
  Check(g1_voice::RouteCommand("今天天气怎么样").type == CommandType::kChat,
        "普通问题进入大模型");
  Check(g1_voice::RouteCommand("机器人为什么会停止").type == CommandType::kChat,
        "普通对话中的停止二字不会误触发动作");
  Check(g1_voice::IsEmergencyStopPhrase("赶紧停下来"), "紧急停止命令");

  if (g_failures != 0) return 1;
  std::cout << "语音文本和命令路由测试全部通过。" << std::endl;
  return 0;
}

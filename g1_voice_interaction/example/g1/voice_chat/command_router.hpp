#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace g1_voice {

enum class CommandType {
  kChat,
  kArmAction,
  kForwardTwoSteps,
  kStopMotion,
};

struct VoiceCommand {
  CommandType type{CommandType::kChat};
  int32_t action_id{0};
  std::string name;
  std::string reply;
  std::chrono::milliseconds hold_time{0};
};

VoiceCommand RouteCommand(const std::string &text);

// 运动期间无需唤醒词也允许触发停止，提高安全性。
bool IsEmergencyStopPhrase(const std::string &text);

}  // namespace g1_voice

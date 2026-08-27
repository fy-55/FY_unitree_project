#pragma once

#include <chrono>
#include <optional>
#include <string>

namespace g1_voice {

std::string Trim(std::string text);
std::optional<std::string> ExtractAsrText(const std::string &payload);

class WakeWordGate {
 public:
  WakeWordGate(std::string wake_word, std::chrono::milliseconds wake_window);

  // 支持两种说法：
  // 1. “小贝同学，向前走两步”；
  // 2. 先说“小贝同学”，再在唤醒窗口内说命令。
  std::optional<std::string> Accept(std::string text);
  bool WaitingForCommand() const;

 private:
  using Clock = std::chrono::steady_clock;

  std::string wake_word_;
  std::chrono::milliseconds wake_window_;
  Clock::time_point active_until_{};
};

}  // namespace g1_voice

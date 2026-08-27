#pragma once

#include "voice_chat/app_config.hpp"

#include <atomic>
#include <deque>
#include <optional>
#include <string>

namespace g1_voice {

struct ChatMessage {
  std::string role;
  std::string content;
};

class LlmClient {
 public:
  LlmClient(LlmConfig config, const std::atomic<bool> &running);

  std::optional<std::string> Ask(const std::string &question,
                                 std::string &error);
  void ClearHistory();

 private:
  LlmConfig config_;
  const std::atomic<bool> &running_;
  std::deque<ChatMessage> history_;
};

}  // namespace g1_voice

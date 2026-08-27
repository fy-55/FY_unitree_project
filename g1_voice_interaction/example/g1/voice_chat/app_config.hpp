#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>

namespace g1_voice {

struct LlmConfig {
  std::string api_url;
  std::string api_key;
  std::string model;
  double temperature{0.6};
  int max_tokens{512};
  bool reasoning_split{false};
  long connect_timeout_seconds{8};
  long request_timeout_seconds{120};
};

struct MotionConfig {
  bool walking_enabled{false};
  float forward_speed{0.12F};
  std::chrono::milliseconds forward_duration{1500};
  std::chrono::milliseconds command_period{100};
};

struct AppConfig {
  LlmConfig llm;
  MotionConfig motion;
  std::string wake_word;
  std::chrono::milliseconds wake_window{8000};
  std::string welcome_text;
  int volume{60};
  std::size_t queue_capacity{2};
  std::chrono::milliseconds duplicate_window{2000};
};

// 从环境变量读取配置。配置不合法时返回空，并在 error 中说明原因。
std::optional<AppConfig> LoadAppConfig(std::string &error);

}  // namespace g1_voice

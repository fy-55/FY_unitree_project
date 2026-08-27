#include "voice_chat/app_config.hpp"

#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <string>

namespace g1_voice {
namespace {

std::string GetEnv(const char *name) {
  const char *value = std::getenv(name);
  return value == nullptr ? "" : value;
}

std::string GetEnvOrDefault(const char *name, const char *default_value) {
  const std::string value = GetEnv(name);
  return value.empty() ? default_value : value;
}

bool IsLocalEndpoint(const std::string &url) {
  return url.find("127.0.0.1") != std::string::npos ||
         url.find("localhost") != std::string::npos;
}

bool ReadBool(const char *name, bool &value, std::string &error) {
  const std::string text = GetEnv(name);
  if (text.empty()) return true;
  if (text == "1" || text == "true" || text == "TRUE" || text == "on") {
    value = true;
    return true;
  }
  if (text == "0" || text == "false" || text == "FALSE" || text == "off") {
    value = false;
    return true;
  }
  error = std::string(name) + " 只能填写 0/1、true/false 或 on/off";
  return false;
}

bool ReadInt(const char *name, int minimum, int maximum, int &value,
             std::string &error) {
  const std::string text = GetEnv(name);
  if (text.empty()) return true;
  try {
    std::size_t parsed = 0;
    const int candidate = std::stoi(text, &parsed);
    if (parsed != text.size() || candidate < minimum || candidate > maximum) {
      throw std::invalid_argument("range");
    }
    value = candidate;
    return true;
  } catch (const std::exception &) {
    error = std::string(name) + " 必须是 " + std::to_string(minimum) + " 到 " +
            std::to_string(maximum) + " 之间的整数";
    return false;
  }
}

bool ReadLong(const char *name, long minimum, long maximum, long &value,
              std::string &error) {
  const std::string text = GetEnv(name);
  if (text.empty()) return true;
  try {
    std::size_t parsed = 0;
    const long candidate = std::stol(text, &parsed);
    if (parsed != text.size() || candidate < minimum || candidate > maximum) {
      throw std::invalid_argument("range");
    }
    value = candidate;
    return true;
  } catch (const std::exception &) {
    error = std::string(name) + " 必须是 " + std::to_string(minimum) + " 到 " +
            std::to_string(maximum) + " 之间的整数";
    return false;
  }
}

bool ReadFloat(const char *name, float minimum, float maximum, float &value,
               std::string &error) {
  const std::string text = GetEnv(name);
  if (text.empty()) return true;
  try {
    std::size_t parsed = 0;
    const float candidate = std::stof(text, &parsed);
    if (parsed != text.size() || candidate < minimum || candidate > maximum) {
      throw std::invalid_argument("range");
    }
    value = candidate;
    return true;
  } catch (const std::exception &) {
    error = std::string(name) + " 必须在安全范围 " +
            std::to_string(minimum) + " 到 " + std::to_string(maximum) + " 内";
    return false;
  }
}

bool ReadDouble(const char *name, double minimum, double maximum, double &value,
                std::string &error) {
  const std::string text = GetEnv(name);
  if (text.empty()) return true;
  try {
    std::size_t parsed = 0;
    const double candidate = std::stod(text, &parsed);
    if (parsed != text.size() || candidate < minimum || candidate > maximum) {
      throw std::invalid_argument("range");
    }
    value = candidate;
    return true;
  } catch (const std::exception &) {
    error = std::string(name) + " 必须在 " + std::to_string(minimum) + " 到 " +
            std::to_string(maximum) + " 之间";
    return false;
  }
}

}  // namespace

std::optional<AppConfig> LoadAppConfig(std::string &error) {
  AppConfig config;
  config.llm.api_url = GetEnvOrDefault(
      "LLM_API_URL", "http://127.0.0.1:11434/v1/chat/completions");
  config.llm.model = GetEnvOrDefault("LLM_MODEL", "qwen3:8b");
  config.llm.api_key = GetEnv("LLM_API_KEY");
  config.wake_word = GetEnv("G1_WAKE_WORD");
  config.welcome_text = GetEnvOrDefault(
      "G1_WELCOME_TEXT",
      "我是小贝同学，欢迎各位领导莅临指导，现在可以跟小贝开始对话了哦。");

  if (config.llm.api_key.empty()) {
    if (IsLocalEndpoint(config.llm.api_url)) {
      config.llm.api_key = "ollama";
    } else {
      error = "云端模型需要设置 LLM_API_KEY，程序已拒绝带空密钥启动";
      return std::nullopt;
    }
  }

  int wake_window_ms = static_cast<int>(config.wake_window.count());
  int walk_duration_ms = static_cast<int>(config.motion.forward_duration.count());
  int command_period_ms = static_cast<int>(config.motion.command_period.count());

  if (!ReadBool("LLM_REASONING_SPLIT", config.llm.reasoning_split, error) ||
      !ReadBool("G1_ENABLE_WALKING", config.motion.walking_enabled, error) ||
      !ReadDouble("LLM_TEMPERATURE", 0.0, 2.0, config.llm.temperature, error) ||
      !ReadInt("LLM_MAX_TOKENS", 16, 4096, config.llm.max_tokens, error) ||
      !ReadLong("LLM_CONNECT_TIMEOUT_SECONDS", 1, 60,
                config.llm.connect_timeout_seconds, error) ||
      !ReadLong("LLM_REQUEST_TIMEOUT_SECONDS", 5, 300,
                config.llm.request_timeout_seconds, error) ||
      !ReadInt("G1_VOLUME", 0, 100, config.volume, error) ||
      !ReadInt("G1_WAKE_WINDOW_MS", 1000, 30000, wake_window_ms, error) ||
      !ReadFloat("G1_WALK_SPEED", 0.05F, 0.30F,
                 config.motion.forward_speed, error) ||
      !ReadInt("G1_WALK_DURATION_MS", 300, 5000, walk_duration_ms, error) ||
      !ReadInt("G1_MOVE_COMMAND_PERIOD_MS", 50, 500, command_period_ms, error)) {
    return std::nullopt;
  }

  config.wake_window = std::chrono::milliseconds(wake_window_ms);
  config.motion.forward_duration = std::chrono::milliseconds(walk_duration_ms);
  config.motion.command_period = std::chrono::milliseconds(command_period_ms);
  return config;
}

}  // namespace g1_voice

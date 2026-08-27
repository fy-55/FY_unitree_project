#include "voice_chat/asr_processor.hpp"

#include <nlohmann/json.hpp>

#include <utility>

namespace g1_voice {
namespace {

std::string TrimCommandPunctuation(std::string text) {
  static const char *prefixes[] = {"，", "。", "！", "？", "：", ",", ".", "!", "?", ":"};
  bool removed = true;
  while (removed) {
    removed = false;
    text = Trim(std::move(text));
    for (const char *prefix : prefixes) {
      const std::string punctuation(prefix);
      if (text.rfind(punctuation, 0) == 0) {
        text.erase(0, punctuation.size());
        removed = true;
        break;
      }
    }
  }
  return Trim(std::move(text));
}

}  // namespace

std::string Trim(std::string text) {
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::optional<std::string> ExtractAsrText(const std::string &payload) {
  try {
    const nlohmann::json message = nlohmann::json::parse(payload);
    if (!message.contains("text") || !message["text"].is_string()) {
      return std::nullopt;
    }

    // 部分G1固件会把可用识别结果标记成 is_final:false，因此这里不按该字段过滤。
    const std::string text = Trim(message["text"].get<std::string>());
    return text.empty() ? std::nullopt
                        : std::optional<std::string>(text);
  } catch (const nlohmann::json::exception &) {
    // 兼容直接发送纯文本的固件。
    const std::string text = Trim(payload);
    return text.empty() ? std::nullopt
                        : std::optional<std::string>(text);
  }
}

WakeWordGate::WakeWordGate(std::string wake_word,
                           std::chrono::milliseconds wake_window)
    : wake_word_(std::move(wake_word)), wake_window_(wake_window) {}

std::optional<std::string> WakeWordGate::Accept(std::string text) {
  text = Trim(std::move(text));
  if (text.empty()) return std::nullopt;
  if (wake_word_.empty()) return text;

  const auto now = Clock::now();
  const auto position = text.find(wake_word_);
  if (position != std::string::npos) {
    text.erase(position, wake_word_.size());
    text = TrimCommandPunctuation(std::move(text));
    if (text.empty()) {
      active_until_ = now + wake_window_;
      return std::nullopt;
    }

    active_until_ = Clock::time_point{};
    return text;
  }

  if (now <= active_until_) {
    active_until_ = Clock::time_point{};
    return text;
  }
  return std::nullopt;
}

bool WakeWordGate::WaitingForCommand() const {
  return !wake_word_.empty() && Clock::now() <= active_until_;
}

}  // namespace g1_voice

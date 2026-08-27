#include "voice_chat/voice_assistant.hpp"

#include "voice_chat/command_router.hpp"

#include <exception>
#include <iostream>
#include <utility>

namespace g1_voice {
namespace {

constexpr const char *kAudioTopic = "rt/audio_msg";

}  // namespace

VoiceAssistant::VoiceAssistant(AppConfig config)
    : config_(std::move(config)),
      robot_(config_, running_),
      llm_(config_.llm, running_),
      wake_gate_(config_.wake_word, config_.wake_window),
      subscriber_(kAudioTopic) {}

VoiceAssistant::~VoiceAssistant() { Stop(); }

bool VoiceAssistant::Start(std::string &error) {
  if (running_.exchange(true)) {
    error = "语音助手已经启动";
    return false;
  }

  try {
    robot_.Initialize();
    subscriber_.InitChannel(
        [this](const void *message) { OnAudioMessage(message); });
    worker_ = std::thread(&VoiceAssistant::WorkerLoop, this);
  } catch (const std::exception &exception) {
    running_.store(false);
    subscriber_.CloseChannel();
    queue_condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    error = std::string("启动语音助手失败：") + exception.what();
    return false;
  }

  std::cout << "大模型地址：" << config_.llm.api_url << std::endl;
  std::cout << "模型：" << config_.llm.model << std::endl;
  std::cout << "唤醒词："
            << (config_.wake_word.empty() ? "未启用（持续对话）"
                                          : config_.wake_word)
            << std::endl;
  std::cout << "行走功能："
            << (config_.motion.walking_enabled ? "已启用" : "已禁用")
            << std::endl;

  robot_.Speak(config_.welcome_text);
  return true;
}

void VoiceAssistant::Stop() {
  if (!running_.exchange(false)) return;

  robot_.RequestStop();
  subscriber_.CloseChannel();
  queue_condition_.notify_all();
  if (worker_.joinable()) worker_.join();
  robot_.Shutdown();
  std::cout << "语音助手已退出。" << std::endl;
}

bool VoiceAssistant::IsRunning() const { return running_.load(); }

void VoiceAssistant::OnAudioMessage(const void *raw_message) {
  if (!running_.load() || raw_message == nullptr || robot_.IsSpeaking()) return;

  const auto *message = static_cast<const AudioMessage *>(raw_message);
  const std::string payload = message->data();
  const auto text = ExtractAsrText(payload);
  if (!text) return;

  std::cout << "[ASR原文] " << payload << std::endl;

  // 运动期间只接收停止命令，其他内容不进入对话队列。
  if (robot_.IsMoving()) {
    if (IsEmergencyStopPhrase(*text)) {
      std::cout << "检测到紧急停止语音。" << std::endl;
      robot_.RequestStop();
      ClearQuestions();
    }
    return;
  }

  std::optional<std::string> accepted;
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    accepted = wake_gate_.Accept(*text);
    if (!accepted) {
      if (wake_gate_.WaitingForCommand()) {
        std::cout << "唤醒成功，请继续说出命令或问题。" << std::endl;
      }
      return;
    }

    if (*accepted == last_question_ &&
        now - last_question_time_ < config_.duplicate_window) {
      return;
    }
    last_question_ = *accepted;
    last_question_time_ = now;
  }

  std::cout << "\n你：" << *accepted << std::endl;
  EnqueueQuestion(std::move(*accepted));
}

void VoiceAssistant::EnqueueQuestion(std::string question) {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (questions_.size() >= config_.queue_capacity) questions_.pop();
    questions_.push(std::move(question));
  }
  queue_condition_.notify_one();
}

void VoiceAssistant::ClearQuestions() {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  while (!questions_.empty()) questions_.pop();
}

void VoiceAssistant::WorkerLoop() {
  while (running_.load()) {
    std::string question;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_condition_.wait(lock, [this] {
        return !running_.load() || !questions_.empty();
      });
      if (!running_.load()) break;
      question = std::move(questions_.front());
      questions_.pop();
    }

    const VoiceCommand command = RouteCommand(question);
    if (command.type != CommandType::kChat) {
      ClearQuestions();
      robot_.Execute(command);
      continue;
    }

    std::cout << "正在请求大模型……" << std::endl;
    std::string error;
    const auto answer = llm_.Ask(question, error);
    if (!answer) {
      std::cerr << error << std::endl;
      if (running_.load()) {
        robot_.Speak("抱歉，大模型暂时没有响应，请检查网络或模型服务。");
      }
      continue;
    }
    robot_.Speak(*answer);
  }
}

}  // namespace g1_voice

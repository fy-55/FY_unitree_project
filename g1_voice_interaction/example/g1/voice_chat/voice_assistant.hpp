#pragma once

#include "voice_chat/app_config.hpp"
#include "voice_chat/asr_processor.hpp"
#include "voice_chat/llm_client.hpp"
#include "voice_chat/robot_controller.hpp"

#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace g1_voice {

class VoiceAssistant {
 public:
  explicit VoiceAssistant(AppConfig config);
  ~VoiceAssistant();

  bool Start(std::string &error);
  void Stop();
  bool IsRunning() const;

  VoiceAssistant(const VoiceAssistant &) = delete;
  VoiceAssistant &operator=(const VoiceAssistant &) = delete;

 private:
  using Clock = std::chrono::steady_clock;
  using AudioMessage = std_msgs::msg::dds_::String_;

  void OnAudioMessage(const void *raw_message);
  void WorkerLoop();
  void EnqueueQuestion(std::string question);
  void ClearQuestions();

  AppConfig config_;
  std::atomic<bool> running_{false};
  RobotController robot_;
  LlmClient llm_;
  WakeWordGate wake_gate_;
  unitree::robot::ChannelSubscriber<AudioMessage> subscriber_;
  std::thread worker_;

  std::mutex input_mutex_;
  std::string last_question_;
  Clock::time_point last_question_time_{};

  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::queue<std::string> questions_;
};

}  // namespace g1_voice

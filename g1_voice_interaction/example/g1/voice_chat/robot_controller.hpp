#pragma once

#include "voice_chat/app_config.hpp"
#include "voice_chat/command_router.hpp"

#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include <atomic>
#include <mutex>
#include <string>

namespace g1_voice {

class RobotController {
 public:
  RobotController(const AppConfig &config, const std::atomic<bool> &running);

  void Initialize();
  bool IsSpeaking() const;
  bool IsMoving() const;

  void Speak(const std::string &text);
  void Execute(const VoiceCommand &command);
  void RequestStop();
  void Shutdown();

 private:
  using AudioClient = unitree::robot::g1::AudioClient;
  using ArmActionClient = unitree::robot::g1::G1ArmActionClient;
  using LocoClient = unitree::robot::g1::LocoClient;

  bool IsArmActionStateSupported(int &fsm_id, int &fsm_mode);
  bool IsWalkingStateSupported(int &fsm_id, int &fsm_mode);
  void ExecuteArmAction(const VoiceCommand &command);
  void ExecuteForwardTwoSteps();
  void ExecuteStop();
  void SleepWhileRunning(std::chrono::milliseconds duration) const;

  const AppConfig &config_;
  const std::atomic<bool> &running_;
  AudioClient audio_client_;
  ArmActionClient arm_client_;
  LocoClient loco_client_;
  std::atomic<bool> speaking_{false};
  std::atomic<bool> moving_{false};
  std::atomic<bool> arm_active_{false};
  std::atomic<bool> stop_requested_{false};
  std::mutex speech_mutex_;
};

}  // namespace g1_voice

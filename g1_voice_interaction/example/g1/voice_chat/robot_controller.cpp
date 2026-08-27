#include "voice_chat/robot_controller.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

namespace g1_voice {
namespace {

using Clock = std::chrono::steady_clock;

class AtomicFlagGuard {
 public:
  explicit AtomicFlagGuard(std::atomic<bool> &flag) : flag_(flag) {
    flag_.store(true);
  }
  ~AtomicFlagGuard() { flag_.store(false); }

  AtomicFlagGuard(const AtomicFlagGuard &) = delete;
  AtomicFlagGuard &operator=(const AtomicFlagGuard &) = delete;

 private:
  std::atomic<bool> &flag_;
};

class StopMoveGuard {
 public:
  explicit StopMoveGuard(unitree::robot::g1::LocoClient &client)
      : client_(client) {}
  ~StopMoveGuard() {
    if (active_) client_.StopMove();
  }

  int32_t StopNow() {
    if (!active_) return 0;
    const int32_t result = client_.StopMove();
    active_ = result != 0;
    return result;
  }

  StopMoveGuard(const StopMoveGuard &) = delete;
  StopMoveGuard &operator=(const StopMoveGuard &) = delete;

 private:
  unitree::robot::g1::LocoClient &client_;
  bool active_{true};
};

std::size_t Utf8CharacterCount(const std::string &text) {
  std::size_t count = 0;
  for (const unsigned char byte : text) {
    if ((byte & 0xC0) != 0x80) ++count;
  }
  return count;
}

std::chrono::milliseconds EstimateSpeechDuration(const std::string &text) {
  const double seconds = std::clamp(Utf8CharacterCount(text) / 4.5 + 1.5,
                                    2.5, 25.0);
  return std::chrono::milliseconds(static_cast<int>(seconds * 1000));
}

}  // namespace

RobotController::RobotController(const AppConfig &config,
                                 const std::atomic<bool> &running)
    : config_(config), running_(running) {}

void RobotController::Initialize() {
  audio_client_.Init();
  audio_client_.SetTimeout(10.0F);

  arm_client_.Init();
  arm_client_.SetTimeout(10.0F);

  loco_client_.Init();
  loco_client_.SetTimeout(10.0F);

  const int32_t volume_result = audio_client_.SetVolume(config_.volume);
  if (volume_result != 0) {
    std::cerr << "设置音量失败，错误码：" << volume_result << std::endl;
  }

  std::cout << "机器人客户端初始化完成，音量=" << config_.volume << std::endl;
}

bool RobotController::IsSpeaking() const { return speaking_.load(); }

bool RobotController::IsMoving() const { return moving_.load(); }

void RobotController::SleepWhileRunning(
    std::chrono::milliseconds duration) const {
  const auto deadline = Clock::now() + duration;
  while (running_.load() && Clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now());
    std::this_thread::sleep_for(
        std::min(remaining, std::chrono::milliseconds(100)));
  }
}

void RobotController::Speak(const std::string &text) {
  if (text.empty() || !running_.load()) return;

  std::lock_guard<std::mutex> lock(speech_mutex_);
  AtomicFlagGuard speaking_guard(speaking_);
  const int32_t result = audio_client_.TtsMaker(text, 0);
  std::cout << "G1：" << text << std::endl;
  std::cout << "TTS返回码：" << result << std::endl;

  if (result == 0) {
    SleepWhileRunning(EstimateSpeechDuration(text));
  } else {
    SleepWhileRunning(std::chrono::milliseconds(500));
  }
  SleepWhileRunning(std::chrono::milliseconds(1200));
}

bool RobotController::IsArmActionStateSupported(int &fsm_id, int &fsm_mode) {
  fsm_id = -1;
  fsm_mode = -1;
  const int32_t id_result = loco_client_.GetFsmId(fsm_id);
  const int32_t mode_result = loco_client_.GetFsmMode(fsm_mode);
  if (id_result != 0 || mode_result != 0) {
    std::cerr << "读取手臂动作FSM失败，ID返回码=" << id_result
              << "，模式返回码=" << mode_result << std::endl;
    return false;
  }
  return fsm_id == 500 || fsm_id == 501 || fsm_id == 802;
}

bool RobotController::IsWalkingStateSupported(int &fsm_id, int &fsm_mode) {
  fsm_id = -1;
  fsm_mode = -1;
  const int32_t id_result = loco_client_.GetFsmId(fsm_id);
  const int32_t mode_result = loco_client_.GetFsmMode(fsm_mode);
  if (id_result != 0 || mode_result != 0) {
    std::cerr << "读取行走FSM失败，ID返回码=" << id_result
              << "，模式返回码=" << mode_result << std::endl;
    return false;
  }
  return (fsm_id == 801 || fsm_id == 802) && fsm_mode == 0;
}

void RobotController::ExecuteArmAction(const VoiceCommand &command) {
  AtomicFlagGuard moving_guard(moving_);
  stop_requested_.store(false);

  int fsm_id = -1;
  int fsm_mode = -1;
  if (!IsArmActionStateSupported(fsm_id, fsm_mode)) {
    std::cerr << "当前状态不支持手臂动作，fsm_id=" << fsm_id
              << "，fsm_mode=" << fsm_mode << std::endl;
    Speak("当前机器人状态不支持手臂动作。");
    return;
  }

  std::cout << "准备执行动作：" << command.name
            << "，动作ID：" << command.action_id << std::endl;
  const int32_t action_result = arm_client_.ExecuteAction(command.action_id);
  if (action_result != 0) {
    std::cerr << "动作执行失败，错误码：" << action_result << std::endl;
    Speak("动作执行失败，请检查机器人状态。");
    return;
  }

  if (command.action_id != 99) arm_active_.store(true);
  Speak(command.reply);

  const auto deadline = Clock::now() + command.hold_time;
  while (running_.load() && !stop_requested_.load() &&
         Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (command.action_id != 99) {
    const int32_t release_result = arm_client_.ExecuteAction(99);
    std::cout << "释放手臂返回码：" << release_result << std::endl;
    arm_active_.store(false);
  }
}

void RobotController::ExecuteForwardTwoSteps() {
  AtomicFlagGuard moving_guard(moving_);
  stop_requested_.store(false);

  if (!config_.motion.walking_enabled) {
    Speak("行走功能当前没有启用。");
    return;
  }

  int fsm_id = -1;
  int fsm_mode = -1;
  if (!IsWalkingStateSupported(fsm_id, fsm_mode)) {
    std::cerr << "当前状态不支持行走，fsm_id=" << fsm_id
              << "，fsm_mode=" << fsm_mode << std::endl;
    Speak("当前机器人状态不支持前进。");
    return;
  }

  Speak("请注意，我要向前走两步了。");
  if (!running_.load()) return;

  std::cout << "开始向前移动（按时间近似两步）：速度="
            << config_.motion.forward_speed
            << "m/s，持续时间=" << config_.motion.forward_duration.count()
            << "ms" << std::endl;

  StopMoveGuard stop_guard(loco_client_);
  int32_t move_result = 0;
  const auto deadline = Clock::now() + config_.motion.forward_duration;
  while (running_.load() && !stop_requested_.load() &&
         Clock::now() < deadline) {
    // false让每条命令只保持1秒；持续运动依靠低频刷新，失联后会自动停。
    move_result = loco_client_.Move(config_.motion.forward_speed, 0.0F, 0.0F,
                                    false);
    if (move_result != 0) break;
    std::this_thread::sleep_for(config_.motion.command_period);
  }

  const int32_t stop_result = stop_guard.StopNow();
  std::cout << "停止移动返回码：" << stop_result << std::endl;
  if (!running_.load()) return;

  if (move_result != 0) {
    std::cerr << "前进指令失败，错误码：" << move_result << std::endl;
    Speak("前进指令执行失败。");
  } else if (stop_result != 0) {
    std::cerr << "停止移动失败，错误码：" << stop_result << std::endl;
    Speak("停止移动指令出现异常。");
  } else if (stop_requested_.load()) {
    Speak("好的，我已经停下了。");
  } else {
    Speak("我已经停下了。");
  }
}

void RobotController::ExecuteStop() {
  stop_requested_.store(true);
  const int32_t stop_result = loco_client_.StopMove();
  const int32_t release_result = arm_client_.ExecuteAction(99);
  arm_active_.store(false);
  std::cout << "停止移动返回码：" << stop_result
            << "，释放手臂返回码：" << release_result << std::endl;
  Speak("好的，我已经停止并复位手臂。");
}

void RobotController::Execute(const VoiceCommand &command) {
  switch (command.type) {
    case CommandType::kArmAction:
      ExecuteArmAction(command);
      break;
    case CommandType::kForwardTwoSteps:
      ExecuteForwardTwoSteps();
      break;
    case CommandType::kStopMotion:
      ExecuteStop();
      break;
    case CommandType::kChat:
      break;
  }
}

void RobotController::RequestStop() { stop_requested_.store(true); }

void RobotController::Shutdown() {
  stop_requested_.store(true);
  const int32_t stop_result = loco_client_.StopMove();
  std::cout << "退出前停止移动返回码：" << stop_result << std::endl;
  if (arm_active_.exchange(false)) {
    const int32_t release_result = arm_client_.ExecuteAction(99);
    std::cout << "退出前释放手臂返回码：" << release_result << std::endl;
  }
}

}  // namespace g1_voice

#include "voice_chat/command_router.hpp"

#include <initializer_list>
#include <string_view>
#include <utility>

namespace g1_voice {
namespace {

bool ContainsAny(std::string_view text,
                 std::initializer_list<std::string_view> keywords) {
  for (const std::string_view keyword : keywords) {
    if (text.find(keyword) != std::string_view::npos) return true;
  }
  return false;
}

VoiceCommand ArmAction(int32_t id, std::string name, std::string reply,
                       std::chrono::milliseconds hold_time =
                           std::chrono::milliseconds(2000)) {
  return {CommandType::kArmAction, id, std::move(name), std::move(reply),
          hold_time};
}

}  // namespace

bool IsEmergencyStopPhrase(const std::string &text) {
  return ContainsAny(text, {"停止", "停下", "立即停止", "赶紧停下", "停止动作",
                            "停下来", "别走了", "不要走了", "紧急停止"});
}

VoiceCommand RouteCommand(const std::string &text) {
  // 普通对话只匹配明确的停止短语；宽松的“停止”只在运动期间使用。
  if (ContainsAny(text, {"立即停止", "停止动作", "停下来", "紧急停止",
                         "手臂复位", "恢复手臂"})) {
    return {CommandType::kStopMotion, 99, "停止并复位", "好的，我已经停止。",
            std::chrono::milliseconds(0)};
  }

  if (ContainsAny(text, {"执行前进两步", "执行向前两步", "向前走两步",
                         "往前走两步"})) {
    return {CommandType::kForwardTwoSteps, 0, "向前走两步", "",
            std::chrono::milliseconds(0)};
  }

  if (ContainsAny(text, {"请挥手", "挥挥手", "挥手动作"})) {
    return ArmAction(25, "胸前挥手", "好的，我来向大家挥挥手。");
  }
  if (ContainsAny(text, {"请鼓掌", "鼓鼓掌", "鼓掌动作"})) {
    return ArmAction(17, "鼓掌", "好的，为大家鼓掌。");
  }
  if (ContainsAny(text, {"右手放胸口", "手放心口", "手放胸前"})) {
    return ArmAction(33, "右手放胸口", "好的。");
  }
  if (ContainsAny(text, {"请举右手", "举起右手", "右手举起来"})) {
    return ArmAction(23, "举起右手", "好的，我举起右手。");
  }
  if (ContainsAny(text, {"表示拒绝", "做拒绝动作", "双手打叉"})) {
    return ArmAction(22, "拒绝动作", "不可以哦。");
  }
  if (ContainsAny(text, {"请和我握手", "握个手", "执行握手动作"})) {
    return ArmAction(27, "握手", "好的，很高兴认识你。",
                     std::chrono::milliseconds(5000));
  }

  return {};
}

}  // namespace g1_voice

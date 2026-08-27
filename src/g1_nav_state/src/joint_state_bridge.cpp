#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace
{
constexpr std::array<const char *, 29> kG1JointNames = {
  "left_hip_pitch_joint",
  "left_hip_roll_joint",
  "left_hip_yaw_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_hip_pitch_joint",
  "right_hip_roll_joint",
  "right_hip_yaw_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint",
  "waist_yaw_joint",
  "waist_roll_joint",
  "waist_pitch_joint",
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint",
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
};
}  // namespace

class JointStateBridge : public rclcpp::Node
{
public:
  JointStateBridge()
  : Node("g1_joint_state_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lf/lowstate");
    output_topic_ = declare_parameter<std::string>("output_topic", "/joint_states");

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      output_topic_, rclcpp::QoS(10).reliable());

    low_state_subscription_ = create_subscription<unitree_hg::msg::LowState>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&JointStateBridge::lowStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Read-only G1 bridge: %s (unitree_hg/LowState) -> %s (sensor_msgs/JointState)",
      input_topic_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Publishing the 29 body joints only; Dex3 finger states use separate hand topics");
  }

private:
  void lowStateCallback(const unitree_hg::msg::LowState::SharedPtr message)
  {
    sensor_msgs::msg::JointState joint_state;
    // Unitree LowState has a hardware tick but no ROS header timestamp. The first
    // live bridge therefore records ROS reception time. Hardware-clock conversion
    // can be added later if timestamp measurements show it is necessary.
    joint_state.header.stamp = now();
    joint_state.name.reserve(kG1JointNames.size());
    joint_state.position.reserve(kG1JointNames.size());
    joint_state.velocity.reserve(kG1JointNames.size());
    joint_state.effort.reserve(kG1JointNames.size());

    for (std::size_t index = 0; index < kG1JointNames.size(); ++index) {
      const auto & motor = message->motor_state[index];
      if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) ||
        !std::isfinite(motor.tau_est))
      {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Rejected LowState tick %u: motor %zu contains a non-finite value",
          message->tick, index);
        return;
      }

      joint_state.name.emplace_back(kG1JointNames[index]);
      joint_state.position.push_back(motor.q);
      joint_state.velocity.push_back(motor.dq);
      joint_state.effort.push_back(motor.tau_est);
    }

    joint_state_publisher_->publish(joint_state);
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateBridge>());
  rclcpp::shutdown();
  return 0;
}

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/sport_mode_state.hpp"

class SportModeOdomAdapter : public rclcpp::Node
{
public:
  SportModeOdomAdapter()
  : Node("g1_sportmode_odom_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odommodestate");
    output_topic_ = declare_parameter<std::string>("output_topic", "/g1/internal_odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    input_velocity_frame_ = declare_parameter<std::string>("input_velocity_frame", "base");
    use_message_timestamp_ = declare_parameter<bool>("use_message_timestamp", true);
    publish_body_tf_ = declare_parameter<bool>("publish_body_tf", true);

    if (base_frame_ == robot_base_frame_) {
      throw std::runtime_error("base_frame and robot_base_frame must be different");
    }
    if (input_velocity_frame_ != "base" && input_velocity_frame_ != "odom") {
      throw std::invalid_argument("input_velocity_frame must be either 'base' or 'odom'");
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::SensorDataQoS());
    body_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    state_subscription_ = create_subscription<unitree_go::msg::SportModeState>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SportModeOdomAdapter::stateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Projecting %s to planar odometry %s -> %s on %s; body frame is %s",
      input_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
      output_topic_.c_str(), robot_base_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Input velocity frame mode: %s (%s)",
      input_velocity_frame_.c_str(),
      input_velocity_frame_ == "odom" ?
      "rotate odom-frame velocity into base frame" :
      "copy velocity already expressed in base frame");
  }

private:
  void stateCallback(const unitree_go::msg::SportModeState::SharedPtr message)
  {
    const auto & position = message->position;
    const auto & velocity = message->velocity;
    const auto & unitree_quaternion = message->imu_state.quaternion;

    for (const auto value : position) {
      if (!std::isfinite(value)) {
        rejectNonFinite();
        return;
      }
    }
    for (const auto value : velocity) {
      if (!std::isfinite(value)) {
        rejectNonFinite();
        return;
      }
    }
    for (const auto value : unitree_quaternion) {
      if (!std::isfinite(value)) {
        rejectNonFinite();
        return;
      }
    }
    if (!std::isfinite(message->yaw_speed)) {
      rejectNonFinite();
      return;
    }
    if (!std::isfinite(message->body_height) || message->body_height <= 0.0F ||
      message->body_height > 2.0F)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected SportModeState containing invalid body_height=%.3f",
        static_cast<double>(message->body_height));
      return;
    }

    // Unitree SportModeState stores its quaternion in [w, x, y, z] order.
    double quaternion_w = unitree_quaternion[0];
    double quaternion_x = unitree_quaternion[1];
    double quaternion_y = unitree_quaternion[2];
    double quaternion_z = unitree_quaternion[3];
    const double quaternion_norm = std::sqrt(
      quaternion_w * quaternion_w + quaternion_x * quaternion_x +
      quaternion_y * quaternion_y + quaternion_z * quaternion_z);

    if (quaternion_norm < 1.0e-6) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected SportModeState containing an invalid quaternion");
      return;
    }

    quaternion_w /= quaternion_norm;
    quaternion_x /= quaternion_norm;
    quaternion_y /= quaternion_norm;
    quaternion_z /= quaternion_norm;

    const tf2::Quaternion full_orientation(
      quaternion_x, quaternion_y, quaternion_z, quaternion_w);
    const double yaw = tf2::getYaw(full_orientation);

    tf2::Quaternion planar_orientation;
    planar_orientation.setRPY(0.0, 0.0, yaw);
    planar_orientation.normalize();

    // Removing the yaw component leaves the body's roll/pitch relative to the
    // ground-projected navigation frame. Quaternion composition is used here
    // instead of copying Euler angles so the two published TFs reconstruct the
    // original full body orientation exactly.
    tf2::Quaternion body_orientation = planar_orientation.inverse() * full_orientation;
    body_orientation.normalize();

    nav_msgs::msg::Odometry odometry;
    if (use_message_timestamp_ && message->stamp.sec >= 0 &&
      message->stamp.nanosec < 1000000000U &&
      (message->stamp.sec != 0 || message->stamp.nanosec != 0))
    {
      odometry.header.stamp.sec = message->stamp.sec;
      odometry.header.stamp.nanosec = message->stamp.nanosec;
    } else {
      odometry.header.stamp = now();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "SportModeState timestamp is unavailable or disabled; using ROS reception time");
    }

    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = position[0];
    odometry.pose.pose.position.y = position[1];
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = tf2::toMsg(planar_orientation);
    // nav_msgs/Odometry requires twist to be expressed in child_frame_id. The
    // Unitree source semantics are selected explicitly until verified on the
    // target G1: copy body-frame input, or rotate odom-frame input into body.
    if (input_velocity_frame_ == "odom") {
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      odometry.twist.twist.linear.x =
        cos_yaw * velocity[0] + sin_yaw * velocity[1];
      odometry.twist.twist.linear.y =
        -sin_yaw * velocity[0] + cos_yaw * velocity[1];
    } else {
      odometry.twist.twist.linear.x = velocity[0];
      odometry.twist.twist.linear.y = velocity[1];
    }
    odometry.twist.twist.linear.z = 0.0;
    odometry.twist.twist.angular.z = message->yaw_speed;

    if (message->error_code != 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "SportModeState reports error_code=%u; forwarding finite state for diagnosis",
        message->error_code);
    }

    odom_publisher_->publish(odometry);

    if (publish_body_tf_) {
      geometry_msgs::msg::TransformStamped body_transform;
      body_transform.header.stamp = odometry.header.stamp;
      body_transform.header.frame_id = base_frame_;
      body_transform.child_frame_id = robot_base_frame_;
      body_transform.transform.translation.x = 0.0;
      body_transform.transform.translation.y = 0.0;
      body_transform.transform.translation.z = message->body_height;
      body_transform.transform.rotation = tf2::toMsg(body_orientation);
      body_tf_broadcaster_->sendTransform(body_transform);
    }

    if (std::abs(static_cast<double>(position[2]) - message->body_height) > 0.5) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring Unitree position.z=%.3f for planar navigation; using body_height=%.3f "
        "for %s -> %s",
        static_cast<double>(position[2]), static_cast<double>(message->body_height),
        base_frame_.c_str(), robot_base_frame_.c_str());
    }
  }

  void rejectNonFinite()
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Rejected SportModeState containing a non-finite pose or velocity value");
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string input_velocity_frame_;
  bool use_message_timestamp_;
  bool publish_body_tf_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> body_tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SportModeOdomAdapter>());
  rclcpp::shutdown();
  return 0;
}

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomTfBridge : public rclcpp::Node
{
public:
  OdomTfBridge()
  : Node("g1_odom_tf_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/g1/internal_odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    if (input_topic_ == output_topic_) {
      throw std::runtime_error("input_topic and output_topic must be different to avoid a loop");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Nav2's odometry consumers request RELIABLE delivery. Keep the robot-facing
    // input subscription on SensorDataQoS(), but republish the normalized ROS
    // odometry with a QoS profile that is compatible with Nav2.
    const auto nav2_odom_qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, nav2_odom_qos);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OdomTfBridge::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for direct odometry %s -> %s on %s; output topic is %s",
      odom_frame_.c_str(), base_frame_.c_str(), input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    // This first bridge is deliberately strict. It may only copy a pose that already
    // describes base_frame in odom_frame. A lidar- or IMU-centred pose needs a real
    // extrinsic conversion and must not be fixed by merely renaming the frames.
    if (message->header.frame_id != odom_frame_ || message->child_frame_id != base_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected odometry with frames '%s' -> '%s'; expected '%s' -> '%s'",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        odom_frame_.c_str(), base_frame_.c_str());
      return;
    }

    const auto & position = message->pose.pose.position;
    auto orientation = message->pose.pose.orientation;

    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Rejected odometry containing non-finite values");
      return;
    }

    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);

    if (quaternion_norm < 1.0e-6) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Rejected odometry containing an invalid quaternion");
      return;
    }

    orientation.x /= quaternion_norm;
    orientation.y /= quaternion_norm;
    orientation.z /= quaternion_norm;
    orientation.w /= quaternion_norm;

    auto output = *message;
    if (output.header.stamp.sec == 0 && output.header.stamp.nanosec == 0) {
      output.header.stamp = now();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Input odometry has a zero timestamp; using the current ROS time for offline testing");
    }
    output.pose.pose.orientation = orientation;
    odom_publisher_->publish(output);

    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header = output.header;
    transform.child_frame_id = output.child_frame_id;
    transform.transform.translation.x = output.pose.pose.position.x;
    transform.transform.translation.y = output.pose.pose.position.y;
    transform.transform.translation.z = output.pose.pose.position.z;
    transform.transform.rotation = output.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfBridge>());
  rclcpp::shutdown();
  return 0;
}

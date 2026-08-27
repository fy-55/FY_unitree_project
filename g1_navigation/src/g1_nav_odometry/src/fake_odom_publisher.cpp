#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class FakeOdomPublisher : public rclcpp::Node
{
public:
  FakeOdomPublisher()
  : Node("g1_fake_odom_publisher")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/g1/internal_odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    linear_speed_ = declare_parameter<double>("linear_speed", 0.10);
    yaw_rate_ = declare_parameter<double>("yaw_rate", 0.10);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    if (publish_rate_ <= 0.0) {
      throw std::runtime_error("publish_rate must be greater than zero");
    }

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::SensorDataQoS());

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FakeOdomPublisher::publishOdom, this));

    RCLCPP_WARN(
      get_logger(),
      "Publishing OFFLINE FAKE odometry on %s; this node does not command a robot",
      output_topic_.c_str());
  }

private:
  void publishOdom()
  {
    const double dt = 1.0 / publish_rate_;
    x_ += linear_speed_ * std::cos(yaw_) * dt;
    y_ += linear_speed_ * std::sin(yaw_) * dt;
    yaw_ += yaw_rate_ * dt;

    nav_msgs::msg::Odometry message;
    message.header.stamp = now();
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = x_;
    message.pose.pose.position.y = y_;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    message.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
    message.twist.twist.linear.x = linear_speed_;
    message.twist.twist.angular.z = yaw_rate_;
    publisher_->publish(message);
  }

  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double linear_speed_;
  double yaw_rate_;
  double publish_rate_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}

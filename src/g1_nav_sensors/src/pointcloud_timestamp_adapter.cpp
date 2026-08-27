#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudTimestampAdapter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  PointCloudTimestampAdapter()
  : Node("g1_pointcloud_timestamp_adapter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/utlidar/cloud_livox_mid360");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/g1/cloud_synced");
    warn_offset_sec_ = declare_parameter<double>("warn_offset_sec", 0.5);

    if (input_topic_ == output_topic_) {
      throw std::invalid_argument(
              "input_topic and output_topic must differ to avoid a point-cloud loop");
    }
    if (!std::isfinite(warn_offset_sec_) || warn_offset_sec_ < 0.0) {
      throw std::invalid_argument("warn_offset_sec must be finite and non-negative");
    }

    // Keep only the newest sensor sample. If this computer becomes busy, stale
    // clouds are dropped instead of building a queue and relabeling old data as new.
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    publisher_ = create_publisher<PointCloud2>(output_topic_, sensor_qos);
    subscription_ = create_subscription<PointCloud2>(
      input_topic_, sensor_qos,
      std::bind(
        &PointCloudTimestampAdapter::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Point-cloud reception-time adapter: %s -> %s (frame and point data unchanged)",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void cloudCallback(PointCloud2::UniquePtr message)
  {
    const auto reception_time = now();
    const bool source_stamp_is_valid =
      message->header.stamp.sec >= 0 &&
      message->header.stamp.nanosec < 1000000000U &&
      (message->header.stamp.sec != 0 || message->header.stamp.nanosec != 0);

    if (source_stamp_is_valid) {
      const rclcpp::Time source_time(
        message->header.stamp, reception_time.get_clock_type());
      const double offset_sec = (reception_time - source_time).seconds();

      if (std::abs(offset_sec) > warn_offset_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Replacing point-cloud source timestamp; reception minus source is %.3f s",
          offset_sec);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Point cloud has no valid source timestamp; using ROS reception time");
    }

    // The message owns its storage in this callback, so the large point buffer is
    // forwarded without an explicit deep copy. Only the ROS timestamp is changed.
    message->header.stamp = reception_time;
    publisher_->publish(std::move(message));
  }

  std::string input_topic_;
  std::string output_topic_;
  double warn_offset_sec_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTimestampAdapter>());
  rclcpp::shutdown();
  return 0;
}

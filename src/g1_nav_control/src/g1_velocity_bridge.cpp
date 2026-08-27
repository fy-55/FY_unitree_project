#include <algorithm>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "unitree_api/msg/request.hpp"

namespace
{
using SteadyClock = std::chrono::steady_clock;
constexpr int64_t kG1VelocityApiId = 7105;
volatile std::sig_atomic_t gShutdownRequested = 0;

void requestShutdown(int)
{
  gShutdownRequested = 1;
}

double ageSeconds(
  const SteadyClock::time_point & now,
  const SteadyClock::time_point & stamp)
{
  return std::chrono::duration<double>(now - stamp).count();
}

bool isFinite(const geometry_msgs::msg::Twist & command)
{
  return std::isfinite(command.linear.x) &&
         std::isfinite(command.linear.y) &&
         std::isfinite(command.angular.z);
}
}  // namespace

class G1VelocityBridge : public rclcpp::Node
{
public:
  G1VelocityBridge()
  : Node("g1_velocity_bridge")
  {
    rcl_interfaces::msg::ParameterDescriptor gate_descriptor;
    gate_descriptor.description = "Read-only switch for physical G1 motion output";
    gate_descriptor.read_only = true;
    enable_motion_ = declare_parameter<bool>(
      "enable_motion", false, gate_descriptor);

    control_rate_ = declare_parameter<double>("control_rate", 20.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.30);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.50);
    max_forward_speed_ = declare_parameter<double>("max_forward_speed", 1.0);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.05);
    command_duration_ = declare_parameter<double>("command_duration", 0.20);
    validateParameters();

    command_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", rclcpp::QoS(1).reliable(),
      std::bind(&G1VelocityBridge::commandCallback, this, std::placeholders::_1));
    scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&G1VelocityBridge::scanCallback, this, std::placeholders::_1));
    request_publisher_ = create_publisher<unitree_api::msg::Request>(
      "/api/sport/request", rclcpp::QoS(10).reliable());
    output_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_g1", rclcpp::QoS(10).reliable());

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_)),
      std::bind(&G1VelocityBridge::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Bridge: /cmd_vel_safe + /scan -> limits/watchdogs -> /api/sport/request (API 7105)");
    if (enable_motion_) {
      RCLCPP_WARN(get_logger(), "PHYSICAL G1 MOTION OUTPUT IS ENABLED");
    } else {
      RCLCPP_WARN(
        get_logger(), "NO-MOTION MODE: API commands will not be published");
    }
  }

  void stopBeforeExit() noexcept
  {
    try {
      timer_->cancel();
      if (!enable_motion_) {
        RCLCPP_INFO(
          get_logger(), "Exit stop skipped because physical output is disabled");
        return;
      }

      RCLCPP_WARN(get_logger(), "Ctrl+C received: publishing zero velocity before exit");
      publishApi(geometry_msgs::msg::Twist());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to publish exit stop: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to publish exit stop: unknown error");
    }
  }

private:
  void validateParameters() const
  {
    const double values[] = {
      control_rate_, command_timeout_, scan_timeout_, max_forward_speed_,
      max_angular_speed_, command_duration_};
    for (const double value : values) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Velocity bridge parameters must be finite and positive");
      }
    }
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    latest_command_ = *message;
    command_valid_ = isFinite(*message);
    have_command_ = true;
    last_command_time_ = SteadyClock::now();
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    scan_valid_ = !message->ranges.empty() &&
      std::isfinite(message->angle_increment) && message->angle_increment > 0.0;
    have_scan_ = true;
    last_scan_time_ = SteadyClock::now();
  }

  void timerCallback()
  {
    const auto now = SteadyClock::now();

    std::string stop_reason;
    if (!have_command_ || ageSeconds(now, last_command_time_) > command_timeout_) {
      stop_reason = "cmd_vel_safe missing or stale";
    } else if (!command_valid_) {
      stop_reason = "cmd_vel_safe contains NaN/Inf";
    } else if (!have_scan_ || !scan_valid_ || ageSeconds(now, last_scan_time_) > scan_timeout_) {
      stop_reason = "scan missing, malformed, or stale";
    }

    if (!stop_reason.empty()) {
      output_command_ = geometry_msgs::msg::Twist();
      reportState(stop_reason);
    } else {
      output_command_ = geometry_msgs::msg::Twist();
      // Initial G1 navigation only permits forward and yaw motion.
      output_command_.linear.x = std::clamp(
        latest_command_.linear.x, 0.0, max_forward_speed_);
      output_command_.angular.z = std::clamp(
        latest_command_.angular.z, -max_angular_speed_, max_angular_speed_);
      reportState("");
    }

    // Mirror the command after watchdogs and final bridge limits. This topic is
    // diagnostic only; publishing it never enables physical motion by itself.
    output_command_publisher_->publish(output_command_);

    if (enable_motion_) {
      publishApi(output_command_);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NO-MOTION output: vx=%.3f m/s, wz=%.3f rad/s",
        output_command_.linear.x, output_command_.angular.z);
    }
  }

  void reportState(const std::string & stop_reason)
  {
    if (stop_reason == last_stop_reason_) {
      return;
    }
    last_stop_reason_ = stop_reason;
    if (stop_reason.empty()) {
      RCLCPP_INFO(get_logger(), "Bridge inputs ready");
    } else {
      RCLCPP_WARN(get_logger(), "Output zero: %s", stop_reason.c_str());
    }
  }

  void publishApi(const geometry_msgs::msg::Twist & command)
  {
    unitree_api::msg::Request request;
    request.header.identity.id = std::chrono::duration_cast<std::chrono::nanoseconds>(
      SteadyClock::now().time_since_epoch()).count();
    request.header.identity.api_id = kG1VelocityApiId;
    request.header.policy.priority = 0;
    request.header.policy.noreply = false;

    std::ostringstream json;
    json << std::fixed << std::setprecision(6)
         << "{\"velocity\":[" << command.linear.x << ",0.000000,"
         << command.angular.z << "],\"duration\":" << command_duration_ << '}';
    request.parameter = json.str();
    request_publisher_->publish(request);
  }

  bool enable_motion_{false};
  bool have_command_{false};
  bool command_valid_{false};
  bool have_scan_{false};
  bool scan_valid_{false};

  std::string last_stop_reason_;

  double control_rate_{20.0};
  double command_timeout_{0.30};
  double scan_timeout_{0.50};
  double max_forward_speed_{0.15};
  double max_angular_speed_{0.25};
  double command_duration_{0.20};

  geometry_msgs::msg::Twist latest_command_;
  geometry_msgs::msg::Twist output_command_;
  SteadyClock::time_point last_command_time_;
  SteadyClock::time_point last_scan_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestShutdown);
  std::signal(SIGTERM, requestShutdown);

  auto bridge = std::make_shared<G1VelocityBridge>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);
  while (rclcpp::ok() && !gShutdownRequested) {
    executor.spin_once(std::chrono::milliseconds(20));
  }

  bridge->stopBeforeExit();
  executor.remove_node(bridge);
  rclcpp::shutdown();
  return 0;
}

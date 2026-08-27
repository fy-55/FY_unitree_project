/**
 * @file b2_obstacle_detector.cpp
 * @brief 障碍物检测节点,订阅雷达和里程计数据检测运动方向上的障碍物
 *
 * 功能说明:
 * - 订阅 /converted_scan 话题获取激光雷达数据 (sensor_msgs/LaserScan)
 * - 订阅 /odom 话题获取里程计数据 (nav_msgs/Odometry)
 * - 基于运动方向检测前方20cm范围内的障碍物
 * - 发布障碍物状态到 /obstacle_ahead 话题 (std_msgs/Bool)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>

class ObstacleDetectorNode : public rclcpp::Node
{
public:
    ObstacleDetectorNode()
        : Node("obstacle_detector")
    {
        // 订阅雷达话题
        rclcpp::QoS qos(10);
        qos.best_effort();
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/converted_scan", qos,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->scanCallback(msg);
            });

        // 订阅里程计话题
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odomCallback(msg);
            });

        // 创建障碍物状态发布者
        obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_ahead", 10);

        // 障碍物检测参数
        obstacle_hold_timer_ = 0.0;  // 障碍物保持计时器
        last_direction_ = 0.0;     // 默认向前
        speed_dead_zone_ = 0.2;    // 速度死区阈值,小于此值认为静止
        min_distance_ = 0.9;       // 基础检测距离

        // 创建10ms周期的障碍物检测发布定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { this->publishObstacleStatus(); });

        RCLCPP_INFO(this->get_logger(), "Obstacle detector started");
        RCLCPP_INFO(this->get_logger(), "Subscribe to /converted_scan and /odom");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_ = msg;
    }

    bool hasObstacleAhead()
    {
        if (!latest_odom_ || !latest_scan_) {
            return false;
        }

        double vx = latest_odom_->twist.twist.linear.x;
        double vy = latest_odom_->twist.twist.linear.y;

        auto scan = latest_scan_;
        double angle_min = scan->angle_min;
        double angle_increment = scan->angle_increment;
        size_t num_ranges = scan->ranges.size();

        double search_angle = std::atan2(vy, vx);
        double speed = std::sqrt(vx * vx + vy * vy);

        // 如果障碍物保持计时器在运行,使用保持的方向,不更新
        if (obstacle_hold_timer_ > 0) {
            search_angle = last_direction_;
        } else if (speed >= speed_dead_zone_) {
            // 正常更新运动方向
            last_direction_ = search_angle;
        } else {
            // 机器人静止且没有保持需求,使用上次的运动方向
            search_angle = last_direction_;
        }

        // 根据运动方向确定检测距离
        double detect_distance = min_distance_;
        double angle_deg = search_angle * 180.0 / M_PI;

        if (speed >= speed_dead_zone_) {
            // 左右运动 (±45度范围内): 增加20cm
            if (std::abs(angle_deg) < 45.0 || std::abs(angle_deg) > 135.0) {
                detect_distance = min_distance_ + 0.2;
            }
            // 后方运动 (90度到135度或-90度到-135度): 增加80cm
            if (std::abs(angle_deg) >= 45.0 && std::abs(angle_deg) <= 135.0) {
                // 实际是侧向或后方运动
                if (std::abs(angle_deg) >= 90.0) {
                    detect_distance = min_distance_ + 0.8;
                }
            }
        }

        for (size_t i = 0; i < num_ranges; i++) {
            double angle = angle_min + static_cast<double>(i) * angle_increment;

            double angle_diff = angle - search_angle;
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            if (std::abs(angle_diff) > M_PI / 8) {  // ±22.5度
                continue;
            }

            float range = scan->ranges[i];

            if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
                continue;
            }

            if (range < detect_distance) {
                return true;
            }
        }

        return false;
    }

    void publishObstacleStatus()
    {
        bool has_obstacle = hasObstacleAhead();

        // 更新时间步长
        static double last_time = this->get_clock()->now().seconds();
        double current_time = this->get_clock()->now().seconds();
        double dt = current_time - last_time;
        last_time = current_time;

        // 如果检测到障碍物,重置保持计时器为1秒
        if (has_obstacle) {
            obstacle_hold_timer_ = 1.0;
        }

        // 如果保持计时器在运行,减少计时器
        if (obstacle_hold_timer_ > 0) {
            obstacle_hold_timer_ -= dt;
        }

        auto msg = std_msgs::msg::Bool();
        msg.data = has_obstacle;
        obstacle_pub_->publish(msg);

        // 仅在检测到障碍物时打印警告
        if (has_obstacle) {
            RCLCPP_WARN(this->get_logger(), "OBSTACLE DETECTED! hold_timer=%.2f", obstacle_hold_timer_);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    // 障碍物保持计时器,检测到障碍物后保持运动方向1秒
    double obstacle_hold_timer_;
    // 速度死区阈值,小于此值认为静止
    double speed_dead_zone_;
    // 基础检测距离
    double min_distance_;
    // 上一次的运动方向
    double last_direction_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
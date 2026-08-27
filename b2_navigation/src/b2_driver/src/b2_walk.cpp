/**
 * @file b2_walk.cpp
 * @brief 订阅速度指令话题,根据几何速度指令控制 B2 机器人移动
 *
 * 该节点监听 geometry_msgs/Twist 类型的速度指令消息,
 * 提取线速度和角速度,并通过宇树高层 API (sport_client_.Move) 控制机器人运动
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include "common/ros2_b2_sport_client.h"

/**
 * @brief B2 行走控制节点类
 *
 * 继承自 rclcpp::Node,用于接收 cmd_vel 速度指令并控制机器人移动
 */
class B2WalkNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数,初始化节点、订阅者和运动客户端
     */
    B2WalkNode() : Node("b2_walk_node"), sport_client_(this)
    {
        this->declare_parameter<std::string>("cmd_vel_topic", "/b2/b2cmd_vel");
        this->declare_parameter<bool>("enable_motion", false);
        this->declare_parameter<double>("linear_deadband", 0.05);
        this->declare_parameter<double>("angular_deadband", 0.10);
        this->declare_parameter<double>("cmd_vel_timeout", 1.0);
        this->declare_parameter<double>("max_linear_speed_x", 0.20);
        this->declare_parameter<double>("max_linear_speed_y", 0.15);
        this->declare_parameter<double>("max_angular_speed", 0.30);
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        enable_motion_ = this->get_parameter("enable_motion").as_bool();
        linear_deadband_ = this->get_parameter("linear_deadband").as_double();
        angular_deadband_ = this->get_parameter("angular_deadband").as_double();
        cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
        max_linear_speed_x_ = this->get_parameter("max_linear_speed_x").as_double();
        max_linear_speed_y_ = this->get_parameter("max_linear_speed_y").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        ValidateParameters();
        last_cmd_time_ = this->now();

        // 创建速度指令话题订阅者,回调函数为 OnCmdVelReceived
        // QoS 设置为 10,保持最新 10 条消息
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->OnCmdVelReceived(msg);
            });

        // 日志输出,提示节点已启动
        RCLCPP_INFO(this->get_logger(), "B2 Walk node started, subscribed to %s",
                    cmd_vel_topic_.c_str());
        if (enable_motion_) {
            RCLCPP_WARN(this->get_logger(), "PHYSICAL B2 MOTION OUTPUT IS ENABLED");
        } else {
            RCLCPP_WARN(this->get_logger(), "NO-MOTION MODE: SportClient commands are disabled");
        }
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (!stopped_ && (this->now() - last_cmd_time_).seconds() > cmd_vel_timeout_) {
                    this->StopRobot("cmd_vel timeout");
                }
            });
    }

private:
    /**
     * @brief /cmd_vel 回调函数,直接发送速度指令到机器人
     * @param msg geometry_msgs/Twist 类型的消息指针
     */
    void OnCmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();

        if (!std::isfinite(msg->linear.x) ||
            !std::isfinite(msg->linear.y) ||
            !std::isfinite(msg->angular.z)) {
            StopRobot("cmd_vel contains NaN/Inf");
            return;
        }

        // 提取速度指令:
        // linear.x  -> 前进/后退速度 vx (m/s)
        // linear.y  -> 侧向移动速度 vy (m/s)
        // angular.z -> 转向角速度 vyaw (rad/s)
        float vx = std::clamp(
            ApplyDeadband(msg->linear.x, linear_deadband_),
            -static_cast<float>(max_linear_speed_x_),
            static_cast<float>(max_linear_speed_x_));
        float vy = std::clamp(
            ApplyDeadband(msg->linear.y, linear_deadband_),
            -static_cast<float>(max_linear_speed_y_),
            static_cast<float>(max_linear_speed_y_));
        float vyaw = std::clamp(
            ApplyDeadband(msg->angular.z, angular_deadband_),
            -static_cast<float>(max_angular_speed_),
            static_cast<float>(max_angular_speed_));

        if (vx == 0.0f && vy == 0.0f && vyaw == 0.0f) {
            StopRobot("cmd_vel deadband");
            return;
        }

        if (!enable_motion_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "NO-MOTION output: vx=%.2f, vy=%.2f, vyaw=%.2f", vx, vy, vyaw);
            return;
        }

        // 构造宇树 API 请求消息
        unitree_api::msg::Request req;

        // 调用宇树运动客户端的 Move 接口
        sport_client_.Move(req, vx, vy, vyaw);
        stopped_ = false;

        // 调试日志输出当前发送的速度值
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Move cmd: vx=%.2f, vy=%.2f, vyaw=%.2f", vx, vy, vyaw);
    }

    float ApplyDeadband(double value, double deadband) const
    {
        return std::abs(value) < deadband ? 0.0f : static_cast<float>(value);
    }

    void ValidateParameters() const
    {
        const double positive_values[] = {
            linear_deadband_, angular_deadband_, cmd_vel_timeout_,
            max_linear_speed_x_, max_linear_speed_y_, max_angular_speed_};
        for (const double value : positive_values) {
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument(
                    "B2 walk safety parameters must be finite and positive");
            }
        }
    }

    void StopRobot(const char * reason)
    {
        if (!enable_motion_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "NO-MOTION stop: %s", reason);
            stopped_ = true;
            return;
        }
        if (stopped_) {
            return;
        }
        unitree_api::msg::Request req;
        sport_client_.StopMove(req);
        stopped_ = true;
        RCLCPP_INFO(this->get_logger(), "StopMove sent: %s", reason);
    }

    // 订阅者,监听速度指令话题
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::string cmd_vel_topic_;
    bool enable_motion_;
    double linear_deadband_;
    double angular_deadband_;
    double cmd_vel_timeout_;
    double max_linear_speed_x_;
    double max_linear_speed_y_;
    double max_angular_speed_;
    rclcpp::Time last_cmd_time_;
    bool stopped_{true};

    // 宇树运动客户端,用于发送高层运动指令
    SportClient sport_client_;
};

/**
 * @brief 程序入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @return 程序退出码
 */
int main(int argc, char **argv)
{
    // 初始化 ROS2 客户端库
    rclcpp::init(argc, argv);

    // 创建 B2 行走控制节点
    auto node = std::make_shared<B2WalkNode>();

    // 创建单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;

    // 将节点添加到执行器
    executor.add_node(node);

    // 启动执行器,阻塞当前线程,持续处理节点回调
    executor.spin();

    // 执行器退出后,关闭 ROS2 客户端库
    rclcpp::shutdown();

    return 0;
}

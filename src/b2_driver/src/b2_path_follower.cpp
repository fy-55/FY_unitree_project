/**
 * @file b2_path_follower.cpp
 * @brief 路径跟随控制器,通过 /cmd_vel 发布速度
 *
 * 功能说明:
 * - 订阅 /plan 话题获取导航路径 (nav_msgs/Path)
 * - 通过 TF 获取机器人当前位置 (map -> base)
 * - 使用纯追踪算法计算速度指令
 * - 发布速度到 /b2cmd_vel 话题
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2/impl/utils.h>
#include <cmath>

/**
 * @brief 路径跟随节点类
 *
 * 使用纯追踪(pure pursuit)算法实现路径跟随:
 * 1. 从路径中选择一个参考点(当前目标waypoint)
 * 2. 计算到目标点的距离和角度
 * 3. 根据距离和角度计算线速度和角速度
 */
class PathFollowerNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数,初始化所有成员并创建发布者、订阅者和定时器
     */
    PathFollowerNode()
        : Node("path_follower"),
          // TF缓冲区,用于存储TF变换历史
          tf_buffer_(this->get_clock()),
          // TF监听器,订阅TF话题并填充缓冲区
          tf_listener_(tf_buffer_),
          // 目标位置(未使用,预留)
          target_x_(0.0), target_y_(0.0), target_yaw_(0.0),
          // PID控制参数: 线性速度P增益, 角速度P增益
          kp_linear_(2.0), kp_angular_(1.0),
          // 速度限制: 最大线速度1.0m/s, 最大角速度0.5rad/s
          max_linear_vel_(0.5), max_angular_vel_(0.5),
          // 路径接收标志
          path_received_(false),
          // 障碍物标志初始化为false
          obstacle_ahead_(false)
    {
        // 创建/cmd_vel话题发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/b2/b2cmd_vel", 10);

        // 订阅导航路径话题
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                this->pathCallback(msg);
            });

        // 订阅障碍物话题
        obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/obstacle_ahead", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->obstacleCallback(msg);
            });

        // 创建50ms周期的控制循环定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { this->controlLoop(); });

        RCLCPP_INFO(this->get_logger(), "Path follower started");
        RCLCPP_INFO(this->get_logger(), "Subscribe to /plan (nav_msgs/Path) for path");

        // 初始化: 清空路径状态
        path_received_ = false;
        current_waypoint_index_ = 0;
        publishStop();
    }

private:
    /**
     * @brief 路径话题回调函数,接收导航规划器发布的路径
     * @param msg 路径消息,包含一系列pose组成的路径点
     */
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // 检查路径是否为空
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }
        // 保存路径并重置路径点索引
        path_ = *msg;
        path_received_ = true;
        current_waypoint_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "Got path with %zu waypoints", path_.poses.size());
    }

    /**
     * @brief 障碍物话题回调函数
     * @param msg 障碍物标志, true表示前方有障碍物
     */
    void obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        obstacle_ahead_ = msg->data;
    }

    /**
     * @brief 获取机器人当前位姿
     * @param x 输出参数,机器人x坐标
     * @param y 输出参数,机器人y坐标
     * @param yaw 输出参数,机器人朝向角(弧度)
     * @return bool 获取成功返回true,失败返回false
     *
     * 通过TF查询map到base的坐标变换,获取机器人在地图中的位置和朝向
     */
    bool getRobotPose(double &x, double &y, double &yaw)
    {
        try {
            // 查询TF变换,从map坐标系到base坐标系的当前变换
            auto transform = tf_buffer_.lookupTransform("map", "base", tf2::TimePointZero);
            RCLCPP_DEBUG(this->get_logger(), "TF lookup success");

            // 提取平移分量作为机器人位置
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;

            // 将四元数转换为欧拉角,提取偏航角(yaw)
            auto q = transform.transform.rotation;
            yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));

            return true;
        } catch (const std::exception &e) {
            // TF查询失败时输出警告,每2秒最多输出一次
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF error: %s", e.what());
            return false;
        }
    }

    /**
     * @brief 控制循环,实现纯追踪算法
     *
     * 纯追踪算法步骤:
     * 1. 获取机器人当前位置
     * 2. 选择当前目标路径点
     * 3. 计算机器人到目标点的距离和角度差
     * 4. 根据角度差计算角速度
     * 5. 根据距离计算线速度
     * 6. 发布速度指令
     */
    void controlLoop()
    {
        // 如果没有收到路径或路径已走完,则返回
        if (!path_received_ || current_waypoint_index_ >= path_.poses.size()) {
            // 路径走完时发送停止指令
            if (path_received_ && current_waypoint_index_ >= path_.poses.size()) {
                RCLCPP_INFO(this->get_logger(), "Path completed!");
                publishStop();
                path_received_ = false;
            }
            return;
        }

        // 获取机器人当前位姿,失败则返回
        double robot_x, robot_y, robot_yaw;
        if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
            return;
        }

        // 避障模式: 有障碍物时停止
        if (obstacle_ahead_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "Obstacle ahead! Stopping...");
            publishStop();
            return;
        }

        // 获取当前目标路径点
        auto &waypoint = path_.poses[current_waypoint_index_].pose;

        // 计算到目标点的距离
        double dx = waypoint.position.x - robot_x;
        double dy = waypoint.position.y - robot_y;
        double dist_to_waypoint = std::sqrt(dx * dx + dy * dy);

        // 计算目标朝向角(从机器人指向目标点的角度)
        double target_yaw = std::atan2(dy, dx);

        // 计算角度差,并归一化到[-PI, PI]范围
        double angle_diff = target_yaw - robot_yaw;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // 使用P控制器计算速度
        double linear_vel;
        double angular_vel;

        // 如果角度差太大(>45度),先原地转向不前进
        if (std::abs(angle_diff) > M_PI / 4) {
            angular_vel = kp_angular_ * angle_diff;
            angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
            linear_vel = 0.0;  // 原地转向
        } else {
            // 角度差小时,全速前进
            angular_vel = kp_angular_ * angle_diff * 0.5;
            angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
            linear_vel = max_linear_vel_;
        }

        // 计算剩余路径点数量
        size_t remaining = path_.poses.size() - current_waypoint_index_;

        // 如果到达当前路径点(距离小于0.1m),切换到下一个路径点
        if (dist_to_waypoint < 0.1) {
            current_waypoint_index_++;
            RCLCPP_INFO(this->get_logger(), "Remaining: %zu | cmd_vel: linear=%.2f, angular=%.2f",
                        remaining, linear_vel, angular_vel);
            return;
        }

        // 构建并发布速度指令
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = linear_vel;
        cmd.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd);

        // 打印当前速度指令和剩余路径点数量
        RCLCPP_INFO(this->get_logger(), "Remaining: %zu | cmd_vel: linear=%.2f, angular=%.2f",
                    remaining, linear_vel, angular_vel);
    }

    /**
     * @brief 发布停止指令,将速度置零
     */
    void publishStop()
    {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    // 速度指令发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    // 路径话题订阅者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    // 控制循环定时器
    rclcpp::TimerBase::SharedPtr timer_;
    // TF缓冲区,存储TF变换历史
    tf2_ros::Buffer tf_buffer_;
    // TF监听器
    tf2_ros::TransformListener tf_listener_;

    // 存储的路径消息
    nav_msgs::msg::Path path_;
    // 路径接收标志
    bool path_received_;
    // 当前目标路径点索引
    size_t current_waypoint_index_ = 0;

    // 目标位置(预留,暂未使用)
    double target_x_, target_y_, target_yaw_;
    // PID控制参数
    double kp_linear_, kp_angular_;
    // 速度限制参数
    double max_linear_vel_, max_angular_vel_;

    // 障碍物检测相关
    bool obstacle_ahead_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
};

/**
 * @brief 主函数,初始化ROS2节点并进入spin循环
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
/**
 * @file b2_keyboard_cmd_vel.cpp
 * @brief 键盘控制 B2 机器人移动
 *
 * 控制说明:
 * - WASD: 固定速度前后左右移动
 * - QE: 固定角速度原地旋转
 * - 空格: 停车
 * - F: 急停(设置急停标志)
 * - R: 恢复平衡站立(BalanceStand)
 * - 1/2/3: 切换速度档位
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <map>
#include "common/ros2_b2_sport_client.h"

class KeyboardCmdVelNode : public rclcpp::Node
{
public:
    KeyboardCmdVelNode()
        : Node("keyboard_cmd_vel_node"),
          sport_client_(this),
          linear_x_(0.0), linear_y_(0.0), angular_z_(0.0),
          emergency_stopped_(false),
          current_speed_level_(1),
          // 速度档位: {移动速度, 侧移速度, 旋转速度}
          speed_levels_({
              {1, {0.15f, 0.1f, 0.25f}},   // 低速档
              {2, {0.30f, 0.20f, 0.50f}},   // 中速档
              {3, {0.50f, 0.35f, 0.80f}},   // 高速档
          })
    {
        this->declare_parameter<std::string>("cmd_vel_topic", "/b2/b2cmd_vel");
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Keyboard cmd_vel controller started, publishing to %s",
                    cmd_vel_topic_.c_str());
        printHelp();

        // 打印当前速度档位
        auto &level = speed_levels_.at(current_speed_level_);
        RCLCPP_INFO(this->get_logger(), "Current speed level: %d (move=%.2f, strafe=%.2f, rotate=%.2f)",
                    current_speed_level_, level.move_speed, level.strafe_speed, level.rotate_speed);
    }

    void printHelp()
    {
        RCLCPP_INFO(this->get_logger(), "=== Control Keys ===");
        RCLCPP_INFO(this->get_logger(), "W/S: forward/backward");
        RCLCPP_INFO(this->get_logger(), "A/D: strafe left/right");
        RCLCPP_INFO(this->get_logger(), "Q/E: rotate left/right");
        RCLCPP_INFO(this->get_logger(), "SPACE: stop");
        RCLCPP_INFO(this->get_logger(), "F: emergency stop");
        RCLCPP_INFO(this->get_logger(), "R: recover to balance stand");
        RCLCPP_INFO(this->get_logger(), "1: low speed (%.2f/%.2f/%.2f)",
                    speed_levels_.at(1).move_speed, speed_levels_.at(1).strafe_speed, speed_levels_.at(1).rotate_speed);
        RCLCPP_INFO(this->get_logger(), "2: medium speed (%.2f/%.2f/%.2f)",
                    speed_levels_.at(2).move_speed, speed_levels_.at(2).strafe_speed, speed_levels_.at(2).rotate_speed);
        RCLCPP_INFO(this->get_logger(), "3: high speed (%.2f/%.2f/%.2f)",
                    speed_levels_.at(3).move_speed, speed_levels_.at(3).strafe_speed, speed_levels_.at(3).rotate_speed);
    }

    char getKey()
    {
        struct termios old_tty, new_tty;
        tcgetattr(STDIN_FILENO, &old_tty);
        new_tty = old_tty;
        new_tty.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tty);
        char ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tty);
        return ch;
    }

    void publishCmdVel()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_x_;
        msg.linear.y = linear_y_;
        msg.angular.z = angular_z_;
        cmd_vel_pub_->publish(msg);
        std::cout << "Level:" << current_speed_level_
                  << " | vx=" << linear_x_ << ", vy=" << linear_y_ << ", vyaw=" << angular_z_ << std::endl;
    }

    /**
     * @brief 急停: 设置急停标志并调用 StopMove
     */
    void triggerEmergencyStop()
    {
        emergency_stopped_ = true;
        unitree_api::msg::Request req;
        sport_client_.StopMove(req);
        RCLCPP_INFO(this->get_logger(), "Emergency stop triggered!");
        std::cout << "EMERGENCY STOP!" << std::endl;
    }

    /**
     * @brief 恢复平衡站立: 调用 BalanceStand
     */
    void recoverBalanceStand()
    {
        unitree_api::msg::Request req;
        sport_client_.BalanceStand(req);
        emergency_stopped_ = false;
        RCLCPP_INFO(this->get_logger(), "Recovered to balance stand!");
        std::cout << "BALANCE STAND RECOVERED" << std::endl;
    }

    void run()
    {
        while (rclcpp::ok()) {
            char key = getKey();

            // 速度档位切换
            if (key == '1' || key == '2' || key == '3') {
                current_speed_level_ = key - '0';
                auto &level = speed_levels_.at(current_speed_level_);
                RCLCPP_INFO(this->get_logger(), "Switched to speed level %d (move=%.2f, strafe=%.2f, rotate=%.2f)",
                            current_speed_level_, level.move_speed, level.strafe_speed, level.rotate_speed);
                continue;
            }

            // 恢复平衡站立(优先级最高)
            if (key == 'r' || key == 'R') {
                recoverBalanceStand();
                continue;
            }

            // 急停(优先级次高)
            if (key == 'f' || key == 'F') {
                triggerEmergencyStop();
                continue;
            }

            // 急停状态下只能恢复,不能移动
            if (emergency_stopped_) {
                std::cout << "EMERGENCY STOPPED! Press R to recover." << std::endl;
                continue;
            }

            auto &level = speed_levels_.at(current_speed_level_);

            // 根据按键设置速度(固定速度,非累加)
            switch (key) {
                case 'w': case 'W':
                    linear_x_ = level.move_speed;   // 前进
                    linear_y_ = 0.0;
                    angular_z_ = 0.0;
                    break;

                case 's': case 'S':
                    linear_x_ = -level.move_speed;  // 后退
                    linear_y_ = 0.0;
                    angular_z_ = 0.0;
                    break;

                case 'a': case 'A':
                    linear_x_ = 0.0;
                    linear_y_ = level.strafe_speed;  // 左侧移
                    angular_z_ = 0.0;
                    break;

                case 'd': case 'D':
                    linear_x_ = 0.0;
                    linear_y_ = -level.strafe_speed; // 右侧移
                    angular_z_ = 0.0;
                    break;

                case 'q': case 'Q':
                    linear_x_ = 0.0;
                    linear_y_ = 0.0;
                    angular_z_ = level.rotate_speed;  // 左旋转
                    break;

                case 'e': case 'E':
                    linear_x_ = 0.0;
                    linear_y_ = 0.0;
                    angular_z_ = -level.rotate_speed; // 右旋转
                    break;

                case ' ':
                    {
                        unitree_api::msg::Request req;
                        sport_client_.StopMove(req);
                        std::cout << "STOP MOVE" << std::endl;
                    }
                    continue;  // 不发布cmd_vel

                default:
                    continue;
            }

            publishCmdVel();
        }
    }

private:
    // 速度档位结构体
    struct SpeedLevel {
        float move_speed;    // 前进/后退速度
        float strafe_speed;  // 侧移速度
        float rotate_speed;  // 旋转速度
    };

    SportClient sport_client_;  // SportClient用于急停和恢复
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::string cmd_vel_topic_;
    float linear_x_;
    float linear_y_;
    float angular_z_;
    bool emergency_stopped_;  // 急停标志
    int current_speed_level_;
    std::map<int, SpeedLevel> speed_levels_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardCmdVelNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}

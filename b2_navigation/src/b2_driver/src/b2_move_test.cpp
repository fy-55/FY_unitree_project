/**
 * @file b2_move_test.cpp
 * @brief 测试 SportClient 的 Move 函数是否可用,带运动时间控制
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "common/ros2_b2_sport_client.h"
#include <iostream>
#include <thread>
#include <chrono>

class MoveTestNode : public rclcpp::Node
{
public:
    MoveTestNode(float duration_sec, float vx, float vy, float vyaw)
        : Node("move_test_node"), sport_client_(this),
          duration_sec_(duration_sec), vx_(vx), vy_(vy), vyaw_(vyaw)
    {
        RCLCPP_INFO(this->get_logger(), "MoveTest node created");
        RCLCPP_INFO(this->get_logger(), "Test params: duration=%.1fs, vx=%.2f, vy=%.2f, vyaw=%.2f",
                    duration_sec_, vx_, vy_, vyaw_);
    }

    bool testMove()
    {
        unitree_api::msg::Request req;
        auto start_time = std::chrono::steady_clock::now();
        int loop_count = 0;

        RCLCPP_INFO(this->get_logger(), "Starting movement test for %.1f seconds...", duration_sec_);

        try {
            while (rclcpp::ok() &&
                   std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - start_time).count() < duration_sec_ * 1000) {
                sport_client_.Move(req, vx_, vy_, vyaw_);
                loop_count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            RCLCPP_INFO(this->get_logger(), "Move() called %d times, test completed!", loop_count);

            // Stop robot
            sport_client_.Move(req, 0.0f, 0.0f, 0.0f);
            RCLCPP_INFO(this->get_logger(), "Robot stopped");
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Move() failed with exception: %s", e.what());
            return false;
        }
    }

private:
    SportClient sport_client_;
    float duration_sec_;
    float vx_;
    float vy_;
    float vyaw_;
};

void printUsage(const char* prog_name)
{
    std::cout << "Usage: " << prog_name << " [duration_sec] [vx] [vy] [vyaw]" << std::endl;
    std::cout << "  duration_sec: Movement duration in seconds (default: 1.0)" << std::endl;
    std::cout << "  vx: Forward/backward speed m/s (default: 0.3)" << std::endl;
    std::cout << "  vy: Left/right speed m/s (default: 0.0)" << std::endl;
    std::cout << "  vyaw: Yaw angular speed rad/s (default: 0.0)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example: " << prog_name << " 5.0 0.5 0.0 0.3" << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    float duration = 1.0f;
    float vx = 0.3f;
    float vy = 0.0f;
    float vyaw = 0.0f;

    if (argc >= 2) {
        if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
            printUsage(argv[0]);
            rclcpp::shutdown();
            return 0;
        }
        duration = std::atof(argv[1]);
    }
    if (argc >= 3) vx = std::atof(argv[2]);
    if (argc >= 4) vy = std::atof(argv[3]);
    if (argc >= 5) vyaw = std::atof(argv[4]);

    auto node = std::make_shared<MoveTestNode>(duration, vx, vy, vyaw);

    std::cout << "=== Testing SportClient::Move() ===" << std::endl;

    bool success = node->testMove();

    rclcpp::shutdown();

    if (success) {
        std::cout << "TEST PASSED" << std::endl;
        return 0;
    } else {
        std::cout << "TEST FAILED" << std::endl;
        return 1;
    }
}
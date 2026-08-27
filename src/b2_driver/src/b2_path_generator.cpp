/**
 * @file b2_path_generator.cpp
 * @brief 生成测试路径,用于验证机器人运动
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class PathGeneratorNode : public rclcpp::Node
{
public:
    PathGeneratorNode()
        : Node("path_generator")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

        this->declare_parameter("type", "line");
        this->declare_parameter("length", 2.0);
        this->declare_parameter("num_points", 20);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->publishPath(); });

        RCLCPP_INFO(this->get_logger(), "Path generator started");
    }

    void publishPath()
    {
        std::string type = this->get_parameter("type").as_string();
        double length = this->get_parameter("length").as_double();
        int num_points = this->get_parameter("num_points").as_int();

        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";

        if (type == "line") {
            // 直线前进
            for (int i = 0; i <= num_points; i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = 0.0;
                pose.pose.position.y = (double)i / num_points * length;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
        } else if (type == "square") {
            // 正方形路径
            double side = length;
            // 下边
            for (int i = 0; i <= num_points; i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = (double)i / num_points * side;
                pose.pose.position.y = 0.0;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
            // 右边
            for (int i = 0; i <= num_points; i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = side;
                pose.pose.position.y = (double)i / num_points * side;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
            // 上边
            for (int i = 0; i <= num_points; i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = side * (1 - (double)i / num_points);
                pose.pose.position.y = side;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
            // 左边
            for (int i = 0; i <= num_points; i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = 0.0;
                pose.pose.position.y = side * (1 - (double)i / num_points);
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
        } else if (type == "circle") {
            // 圆形路径
            double radius = length / (2 * M_PI);
            for (int i = 0; i <= num_points; i++) {
                double angle = 2 * M_PI * i / num_points;
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = radius + radius * std::cos(angle);
                pose.pose.position.y = radius + radius * std::sin(angle);
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
        } else if (type == "figure8") {
            // 8字形路径
            for (int i = 0; i <= num_points; i++) {
                double t = 2 * M_PI * i / num_points;
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = length * std::sin(t);
                pose.pose.position.y = length * std::sin(t) * std::cos(t);
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
        }

        path_pub_->publish(path);

        RCLCPP_INFO(this->get_logger(), "Published %s path with %zu points",
                    type.c_str(), path.poses.size());

        timer_->cancel();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
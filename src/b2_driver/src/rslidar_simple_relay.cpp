/**
 * @file rslidar_simple_relay.cpp
 * @brief 雷达点云转发,支持PointCloud2和LaserScan
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class SimpleRelayNode : public rclcpp::Node
{
public:
    SimpleRelayNode() : Node("rslidar_simple_relay")
    {
        RCLCPP_INFO(this->get_logger(), "rslidar_simple_relay started");

        // PointCloud2 订阅和发布
        pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rslidar_pcl", 10);
        pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "rt/rslidar_points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->pclCallback(msg);
            });

        // LaserScan 订阅和发布
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("rslidar_scan", 10);
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/converted_scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->scanCallback(msg);
            });
    }

private:
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        static int pcl_count = 0;
        pcl_count++;

        RCLCPP_INFO_ONCE(this->get_logger(), "First PCL received! width=%u height=%u",
                         msg->width, msg->height);

        if (pcl_count % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "PCL: width=%u height=%u", msg->width, msg->height);
        }

        pcl_publisher_->publish(*msg);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        static int scan_count = 0;
        scan_count++;

        RCLCPP_INFO_ONCE(this->get_logger(), "First LaserScan received! ranges=%zu angle_min=%.2f angle_max=%.2f",
                         msg->ranges.size(), msg->angle_min, msg->angle_max);

        if (scan_count % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "LaserScan: ranges=%zu angle_min=%.2f angle_max=%.2f",
                        msg->ranges.size(), msg->angle_min, msg->angle_max);
        }

        scan_publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleRelayNode>());
    rclcpp::shutdown();
    return 0;
}
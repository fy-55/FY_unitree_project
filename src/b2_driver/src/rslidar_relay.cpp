#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <chrono>
#include <cmath>

class RelayNode : public rclcpp::Node
{
public:
    RelayNode() : Node("rslidar_relay")
    {
        this->declare_parameter("filter_empty", true);
        this->declare_parameter("filter_range_min", 0.0);  // 最小距离(m)
        this->declare_parameter("filter_range_max", 100.0);  // 最大距离(m)
        this->declare_parameter("filter_left_right", 0.22);  // 左右过滤宽度(m), 默认22cm
        this->declare_parameter("filter_behind", 0.8);  // 后方过滤长度(m), 默认80cm
        this->declare_parameter("filter_front", 0.05);  // 前方过滤长度(m), 默认5cm
        this->declare_parameter("filter_side_range", 0.2);  // 左右90度角范围内距离过滤(m), 默认20cm

        filter_empty_ = this->get_parameter("filter_empty").as_bool();
        range_min_ = this->get_parameter("filter_range_min").as_double();
        range_max_ = this->get_parameter("filter_range_max").as_double();
        filter_left_right_ = this->get_parameter("filter_left_right").as_double();
        filter_behind_ = this->get_parameter("filter_behind").as_double();
        filter_front_ = this->get_parameter("filter_front").as_double();
        filter_side_range_ = this->get_parameter("filter_side_range").as_double();

        RCLCPP_INFO(this->get_logger(), "rslidar_relay started, filter_empty=%s", filter_empty_ ? "true" : "false");
        //不要在话题名称中加命名空间
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rslidar", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->relayCallback(msg);
            });
    }

private:
    bool isPointValid(const float* data, uint32_t point_step)
    {
        // 假设PointXYZ格式: x, y, z (各4字节)
        if (point_step >= 12) {
            float x = data[0];
            float y = data[1];
            float z = data[2];

            // 检查是否为NaN
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                return false;
            }

            // 检查是否全为0
            if (x == 0.0f && y == 0.0f && z == 0.0f) {
                return false;
            }

            // 检查距离范围
            float range = std::sqrt(x*x + y*y + z*z);
            if (range < range_min_ || range > range_max_) {
                return false;
            }

            // 过滤雷达后方80cm × 15cm方形区域内的点
            // 从雷达中心(x=0,y=0)开始，后方延伸80cm，宽15cm的方形区域
            // 即: x < 0 && x >= -0.8 && |y| < 0.15
            if (x < 0 && x >= -filter_behind_ && std::abs(y) < filter_left_right_) {
                return false;
            }

            // 过滤前方5cm范围内的点
            if (x > 0 && x <= filter_front_) {
                return false;
            }

            // 过滤左右90度角范围内20cm的点
            // 即: atan2(|y|, x) 在 [π/4, π/2] 且 range < 0.2m
            float abs_y = std::abs(y);
            if (abs_y > 0 && x >= 0) {
                float angle = std::atan2(abs_y, x);
                if (angle >= 0.785398f && range < filter_side_range_) {  // 45度 = π/4 ≈ 0.7854
                    return false;
                }
            }
        }
        return true;
    }

    void relayCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 跳过空点云
        if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {
            return;
        }

        // 更改时间戳为本机时间
        auto now = std::chrono::system_clock::now();
        auto nowSec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        auto nowNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000ULL;

        if (!filter_empty_) {
            // 不过滤,直接转发
            auto pcl_out = *msg;
            pcl_out.header.stamp.sec = static_cast<int32_t>(nowSec);
            pcl_out.header.stamp.nanosec = static_cast<uint32_t>(nowNsec);
            publisher_->publish(pcl_out);
            return;
        }

        // 过滤空白点
        uint32_t point_step = msg->point_step;
        uint32_t width = msg->width;
        uint32_t height = msg->height;
        uint32_t num_points = width * height;

        // 计算有效点的数量
        uint32_t valid_count = 0;
        const uint8_t* data_ptr = msg->data.data();

        for (uint32_t i = 0; i < num_points; i++) {
            if (isPointValid(reinterpret_cast<const float*>(data_ptr + i * point_step), point_step)) {
                valid_count++;
            }
        }

        if (valid_count == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "All points filtered out, skipping publish");
            return;
        }

        // 创建新的PointCloud2
        sensor_msgs::msg::PointCloud2 pcl_out;
        pcl_out.header = msg->header;
        pcl_out.header.stamp.sec = static_cast<int32_t>(nowSec);
        pcl_out.header.stamp.nanosec = static_cast<uint32_t>(nowNsec);
        pcl_out.height = 1;
        pcl_out.width = valid_count;
        pcl_out.point_step = point_step;
        pcl_out.row_step = valid_count * point_step;
        pcl_out.fields = msg->fields;
        pcl_out.is_bigendian = msg->is_bigendian;
        pcl_out.is_dense = true;

        // 分配内存并复制有效点
        pcl_out.data.resize(valid_count * point_step);
        uint8_t* out_ptr = pcl_out.data.data();
        uint32_t valid_written = 0;

        for (uint32_t i = 0; i < num_points; i++) {
            if (isPointValid(reinterpret_cast<const float*>(data_ptr + i * point_step), point_step)) {
                std::memcpy(out_ptr + valid_written * point_step,
                           data_ptr + i * point_step,
                           point_step);
                valid_written++;
            }
        }

        static int print_count = 0;
        if (print_count++ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Filtered: %u/%u points kept",
                        valid_written, num_points);
        }

        publisher_->publish(pcl_out);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    bool filter_empty_;
    double range_min_;
    double range_max_;
    double filter_left_right_;
    double filter_behind_;
    double filter_front_;
    double filter_side_range_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelayNode>());
    rclcpp::shutdown();
    return 0;
}
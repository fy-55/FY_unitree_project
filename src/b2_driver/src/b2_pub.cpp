#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/motor_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"


using namespace std::chrono_literals;

class OdomPub: public rclcpp::Node{
public:
    OdomPub() : Node("OdomPub"){
        RCLCPP_INFO(this->get_logger(),"发布OdomPub节点:");
        this->declare_parameter("publish_odom_tf",true);
        this->declare_parameter("odom_frame","odom");
        this->declare_parameter("base_frame","base");

        rclcpp::QoS qos(rclcpp::KeepLast(1000));

        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);


        publish_odom_tf = this->get_parameter("publish_odom_tf").as_bool();
        odom_frame = this->get_parameter("odom_frame").as_string();
        base_frame = this->get_parameter("base_frame").as_string();
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        //timer = this->create_wall_timer(10ms,std::bind(&OdomPub::timer_callback,this));
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
        Sport_sub = this->create_subscription<unitree_go::msg::SportModeState>(
        "lf/sportmodestate",
        qos,
        std::bind(&OdomPub::sub_callback,this,std::placeholders::_1));


    }
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr Sport_sub;
    rclcpp::TimerBase::SharedPtr timer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool publish_odom_tf;
    std::string odom_frame, base_frame;
    

    void timer_callback(){
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();

        odom.header.frame_id = "odom";
        odom.child_frame_id = "base";

        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.w = 0.0;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = 0.0;

        // 设置线速度
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;

        // 设置角速度
        odom.twist.twist.angular.z = 0.0;

        pub_->publish(odom);
    }

    void sub_callback(unitree_go::msg::SportModeState::SharedPtr data){
                // 创建里程计消息
        nav_msgs::msg::Odometry odom_msg;

        // 设置时间戳
        // odom_msg.header.stamp.sec = data->stamp.sec;
        // odom_msg.header.stamp.nanosec = data->stamp.nanosec;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id = base_frame;

        // 设置位置
        odom_msg.pose.pose.position.x = data->position[0];
        odom_msg.pose.pose.position.y = data->position[1];
        odom_msg.pose.pose.position.z = data->position[2];

        // 设置姿态
        odom_msg.pose.pose.orientation.w = data->imu_state.quaternion[0];
        odom_msg.pose.pose.orientation.x = data->imu_state.quaternion[1];
        odom_msg.pose.pose.orientation.y = data->imu_state.quaternion[2];
        odom_msg.pose.pose.orientation.z = data->imu_state.quaternion[3];

        // 设置线速度
        odom_msg.twist.twist.linear.x = data->velocity[0];
        odom_msg.twist.twist.linear.y = data->velocity[1];
        odom_msg.twist.twist.linear.z = data->velocity[2];

        // 设置角速度
        odom_msg.twist.twist.angular.z = data->yaw_speed;

        // 发布里程计消息
        pub_->publish(odom_msg);

        // 根据参数选择是否发布坐标变换
        if (publish_odom_tf) {
            geometry_msgs::msg::TransformStamped transformStamped;

            // 设置时间戳
            // transformStamped.header.stamp.sec = data->stamp.sec;
            // transformStamped.header.stamp.nanosec = data->stamp.nanosec;
            transformStamped.header.stamp = this->now();
            transformStamped.header.frame_id = odom_frame;
            transformStamped.child_frame_id = base_frame;

            // 设置平移
            transformStamped.transform.translation.x = data->position[0];
            transformStamped.transform.translation.y = data->position[1];
            transformStamped.transform.translation.z = data->position[2];

            // 设置旋转
            transformStamped.transform.rotation = odom_msg.pose.pose.orientation;

            // 发布坐标变换
            tf_broadcaster_->sendTransform(transformStamped);
        }
    }
};



int main(int argc,char ** args){
    rclcpp::init(argc,args);
    rclcpp::spin(std::make_shared<OdomPub>());
    rclcpp::shutdown();
    return 0;
}
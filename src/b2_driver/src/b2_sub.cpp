#include "rclcpp/rclcpp.hpp"
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

class OdomSub: public rclcpp::Node{
public:
    OdomSub() : Node("OdomSub"){
        RCLCPP_INFO(this->get_logger(),"发布OdomSub节点:");
        this->sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&OdomSub::odom_callback,this,std::placeholders::_1));
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    void odom_callback(nav_msgs::msg::Odometry const& msg){
        nav_msgs::msg::Odometry odom;
        odom = msg;
        RCLCPP_INFO(rclcpp::get_logger("odom_printer"), 
                    "odom.pose.pose.position: x=%f, y=%f, z=%f", 
                    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        RCLCPP_INFO(rclcpp::get_logger("odom_printer"), 
                    "odom.pose.pose.orientation: w=%f, x=%f, y=%f, z=%f", 
                    odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        RCLCPP_INFO(rclcpp::get_logger("odom_printer"), 
                    "odom.twist.twist.linear: x=%f, y=%f, z=%f", 
                    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
        RCLCPP_INFO(rclcpp::get_logger("odom_printer"), 
                    "odom.twist.twist.angular: z=%f", 
                    odom.twist.twist.angular.z);
    }
    
};


int main(int argc,char ** args){
    rclcpp::init(argc,args);
    rclcpp::spin(std::make_shared<OdomSub>());
    rclcpp::shutdown();
    return 0;
}
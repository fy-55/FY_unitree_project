from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pub_node = Node(package='b2_driver', executable='b2_pub')
    sub_node = Node(package='b2_driver', executable='b2_sub')
    return LaunchDescription([pub_node, sub_node])

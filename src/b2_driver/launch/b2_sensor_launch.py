from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 静态坐标变换发布节点: rslidar -> base
    # x=0.2m (向前), y=0, z=0.2m (向上)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.2', '0', '0', '0', 'base', 'rslidar']
    )

    # b2_pub 节点
    b2_pub_node = Node(
        package='b2_driver',
        executable='b2_pub'
    )

    # rslidar_relay 节点
    rslidar_relay_node = Node(
        package='b2_driver',
        executable='rslidar_relay'
    )

    return LaunchDescription([
        b2_pub_node,
        rslidar_relay_node,
        static_tf_node
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 定义 pointcloud_to_laserscan 节点
    pcl_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',

        # 话题重映射
        remappings=[
            ('cloud_in', '/rslidar'),  # 输入：3D点云
            ('scan', '/converted_scan')       # 输出：2D激光
        ],

        # 核心参数配置
        parameters=[{
            'target_frame': 'base',          # 投影到机器人底盘坐标系
            'transform_tolerance': 0.1,      # TF变换容差
            'use_sim_time': False,           # 实际硬件运行设为 False

            # --- 关键修改：切片高度控制 ---
            # 取机器人周围适合 2D 建图的高度带，避免地面和过高点进入 LaserScan。
            'min_height': -0.2,
            'max_height': 0.6,

            # --- 关键修改：视角与分辨率 ---
            'angle_min': -3.1415926,         # -180度
            'angle_max': 3.1415926,          # 180度
            'angle_increment': 0.0043,       # 约0.25度/点，提高点云密度感
            'scan_time': 0.1,                # 10Hz

            # --- 关键修改：距离控制 ---
            'range_min': 0.1,                # 必须为正数！避开机器人自身
            'range_max': 15.0,               # 探测距离设为50米
            'use_inf': True,                 # 超出范围显示为 inf
            'inf_epsilon': 1.0
        }]
    )

    return LaunchDescription([
        pcl_to_scan_node
    ])

"""Launch SLAM Toolbox localization with an explicit serialized map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the localization launch description."""
    package_share = get_package_share_directory('b2_navigation')
    params_file = LaunchConfiguration('params_file')
    map_file_name = LaunchConfiguration('map_file_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'params', 'slam_toolbox_localization.yaml'),
            description='SLAM Toolbox localization parameter file'),
        DeclareLaunchArgument(
            'map_file_name',
            description='Serialized SLAM Toolbox map base path'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params_file,
                {
                    'map_file_name': map_file_name,
                    'use_sim_time': use_sim_time,
                },
            ]),
    ])

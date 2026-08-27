from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    synced_cloud_topic = LaunchConfiguration("synced_cloud_topic")
    raw_scan_topic = LaunchConfiguration("raw_scan_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    target_frame = LaunchConfiguration("target_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")

    pointcloud_timestamp_adapter = Node(
        package="g1_nav_sensors",
        executable="pointcloud_timestamp_adapter",
        name="g1_pointcloud_timestamp_adapter",
        output="screen",
        parameters=[
            {
                "input_topic": cloud_topic,
                "output_topic": synced_cloud_topic,
                "warn_offset_sec": 0.5,
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            }
        ],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="g1_pointcloud_to_laserscan",
        output="screen",

        remappings=[
            ("cloud_in", synced_cloud_topic),
            ("scan", raw_scan_topic),
        ],

        parameters=[
            {
                # 将点云转换到机器人导航基准坐标系后再切片
                "target_frame": target_frame,
                "transform_tolerance": 0.1,

                # 相对于 base_footprint 的离地高度范围
                "min_height": 0.20,
                "max_height": 1.50,

                # 生成360度二维扫描
                "angle_min": -3.1415926,
                "angle_max": 3.1415926,
                "angle_increment": 0.00872665,
                "scan_time": 0.1,

                # 忽略机器人身体内部和过远的点
                "range_min": 0.30,
                "range_max": 15.0,

                "use_inf": True,
                "inf_epsilon": 1.0,

                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            }
        ],
    )

    scan_speckle_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="g1_scan_speckle_filter",
        output="screen",
        remappings=[
            ("scan", raw_scan_topic),
            ("scan_filtered", scan_topic),
        ],
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("g1_nav_sensors"),
                    "config",
                    "scan_speckle_filter.yaml",
                ]
            ),
            {
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/utlidar/cloud_livox_mid360",
                description="Raw physical PointCloud2 topic",
            ),

            DeclareLaunchArgument(
                "synced_cloud_topic",
                default_value="/g1/cloud_synced",
                description="PointCloud2 topic restamped with ROS reception time",
            ),

            DeclareLaunchArgument(
                "raw_scan_topic",
                default_value="/scan_raw",
                description="Unfiltered LaserScan produced from the point cloud",
            ),

            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="Filtered LaserScan used by SLAM and navigation",
            ),

            DeclareLaunchArgument(
                "target_frame",
                default_value="base_footprint",
                description="Frame used to filter and project the point cloud",
            ),

            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulator clock",
            ),

            pointcloud_timestamp_adapter,
            pointcloud_to_laserscan_node,
            scan_speckle_filter,
        ]
    )

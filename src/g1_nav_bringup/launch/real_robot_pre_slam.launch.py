from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lowstate_topic = LaunchConfiguration("lowstate_topic")
    sport_state_topic = LaunchConfiguration("sport_state_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    raw_scan_topic = LaunchConfiguration("raw_scan_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    navigation_base_frame = LaunchConfiguration("navigation_base_frame")
    robot_base_frame = LaunchConfiguration("robot_base_frame")
    input_velocity_frame = LaunchConfiguration("input_velocity_frame")
    use_message_timestamp = LaunchConfiguration("use_message_timestamp")
    use_sim_time = LaunchConfiguration("use_sim_time")

    description_share = Path(
        get_package_share_directory("g1_nav_description")
    )
    sensors_share = Path(get_package_share_directory("g1_nav_sensors"))

    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(description_share / "launch" / "model.launch.py")
        ),
        launch_arguments={
            "use_joint_state_gui": "false",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    joint_state_bridge = Node(
        package="g1_nav_state",
        executable="joint_state_bridge",
        name="g1_joint_state_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": lowstate_topic,
                "output_topic": "/joint_states",
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    sportmode_odom_adapter = Node(
        package="g1_nav_odometry",
        executable="sportmode_odom_adapter",
        name="g1_sportmode_odom_adapter",
        output="screen",
        parameters=[
            {
                "input_topic": sport_state_topic,
                "output_topic": "/g1/internal_odom",
                "odom_frame": odom_frame,
                "base_frame": navigation_base_frame,
                "robot_base_frame": robot_base_frame,
                "input_velocity_frame": ParameterValue(
                    input_velocity_frame, value_type=str
                ),
                "publish_body_tf": True,
                "use_message_timestamp": ParameterValue(
                    use_message_timestamp, value_type=bool
                ),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    odom_tf_bridge = Node(
        package="g1_nav_odometry",
        executable="odom_tf_bridge",
        name="g1_odom_tf_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": "/g1/internal_odom",
                "output_topic": "/odom",
                "odom_frame": odom_frame,
                "base_frame": navigation_base_frame,
                "publish_tf": True,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sensors_share / "launch" / "pointcloud_to_laserscan.launch.py")
        ),
        launch_arguments={
            "cloud_topic": cloud_topic,
            "raw_scan_topic": raw_scan_topic,
            "scan_topic": scan_topic,
            "target_frame": navigation_base_frame,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lowstate_topic",
                default_value="/lf/lowstate",
                description="Unitree G1 LowState topic",
            ),
            DeclareLaunchArgument(
                "sport_state_topic",
                default_value="/odommodestate",
                description="Unitree SportModeState topic used as local odometry",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/utlidar/cloud_livox_mid360",
                description="Physical Mid360 PointCloud2 topic",
            ),
            DeclareLaunchArgument(
                "raw_scan_topic",
                default_value="/scan_raw",
                description="Unfiltered planar LaserScan for diagnostics",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="Filtered planar LaserScan used downstream",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Local odometry frame",
            ),
            DeclareLaunchArgument(
                "navigation_base_frame",
                default_value="base_footprint",
                description="Planar base frame tracked by 2D odometry",
            ),
            DeclareLaunchArgument(
                "robot_base_frame",
                default_value="base_link",
                description="Physical pelvis-aligned URDF root frame",
            ),
            DeclareLaunchArgument(
                "input_velocity_frame",
                default_value="base",
                description=(
                    "Frame used by Unitree velocity components: base copies them; "
                    "odom rotates them into navigation_base_frame"
                ),
            ),
            DeclareLaunchArgument(
                "use_message_timestamp",
                default_value="false",
                description="Preserve the Unitree SportModeState timestamp",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulator /clock instead of the physical clock",
            ),
            model_launch,
            joint_state_bridge,
            sportmode_odom_adapter,
            odom_tf_bridge,
            scan_launch,
        ]
    )

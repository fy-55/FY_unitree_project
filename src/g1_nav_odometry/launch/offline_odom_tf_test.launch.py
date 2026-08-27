from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(
        get_package_share_directory("g1_nav_description")
    )

    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(description_share / "launch" / "model.launch.py")
        ),
        launch_arguments={
            "use_joint_state_gui": "false",
            "use_sim_time": "false",
        }.items(),
    )

    # Headless replacement for the GUI: publish zero/default joint angles so
    # robot_state_publisher can calculate every movable link during offline tests.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="offline_joint_state_publisher",
        output="screen",
    )

    fake_odom_publisher = Node(
        package="g1_nav_odometry",
        executable="fake_odom_publisher",
        name="g1_fake_odom_publisher",
        output="screen",
        parameters=[
            {
                "output_topic": "/g1/internal_odom",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "linear_speed": 0.10,
                "yaw_rate": 0.10,
                "publish_rate": 20.0,
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
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "publish_tf": True,
            }
        ],
    )

    # The fake source has no physical body-height measurement. Supply a fixed
    # standing height only for this offline TF-chain test.
    fake_body_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="offline_base_footprint_to_base_link",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.78",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
    )

    return LaunchDescription(
        [
            model_launch,
            joint_state_publisher,
            fake_odom_publisher,
            odom_tf_bridge,
            fake_body_tf,
        ]
    )

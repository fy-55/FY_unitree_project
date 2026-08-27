"""Start Gazebo Classic with the planar G1 navigation proxy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    description_share = get_package_share_directory("g1_nav_description")
    sensors_share = get_package_share_directory("g1_nav_sensors")
    simulation_share = get_package_share_directory("g1_nav_sim")

    model_paths = [os.path.join(simulation_share, "models")]
    inherited_model_path = os.environ.get("GAZEBO_MODEL_PATH")
    if inherited_model_path:
        model_paths.append(inherited_model_path)

    resource_paths = [os.path.dirname(description_share)]
    inherited_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH")
    if inherited_resource_path:
        resource_paths.append(inherited_resource_path)
    gazebo_classic_resources = "/usr/share/gazebo-11"
    if os.path.isdir(gazebo_classic_resources):
        resource_paths.append(gazebo_classic_resources)

    world = LaunchConfiguration("world")
    verbose = LaunchConfiguration("verbose")
    gui = LaunchConfiguration("gui")

    base_footprint_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_base_footprint_to_base_link",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.794272897",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_share,
                "launch",
                "model.launch.py",
            )
        ),
        launch_arguments={
            "use_joint_state_gui": "false",
            "use_sim_time": "true",
        }.items(),
    )

    # The Gazebo shell is rigid, so all official movable joints stay at their
    # URDF zero positions. Publishing those constant positions lets
    # robot_state_publisher create the complete official G1 + dexterous-hands
    # TF tree for RViz (29 body joints plus the hand joints in this URDF).
    simulated_joint_states = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="sim_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "rate": 30.0,
                "use_sim_time": True,
            }
        ],
    )

    scan_conversion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sensors_share,
                "launch",
                "pointcloud_to_laserscan.launch.py",
            )
        ),
        launch_arguments={
            "cloud_topic": "/utlidar/cloud_livox_mid360",
            "synced_cloud_topic": "/g1/cloud_synced",
            "scan_topic": "/scan",
            "target_frame": "base_link",
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                simulation_share,
                "worlds",
                "g1_factory.world",
            ]),
            description="Absolute path to the Gazebo world file",
        ),
        DeclareLaunchArgument("verbose", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            value=os.pathsep.join(model_paths),
        ),
        SetEnvironmentVariable(
            "GAZEBO_RESOURCE_PATH",
            value=os.pathsep.join(resource_paths),
        ),
        robot_description,
        simulated_joint_states,
        base_footprint_to_base_link,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
            ),
            launch_arguments={
                "world": world,
                "verbose": verbose,
                "gui": gui,
            }.items(),
        ),
        scan_conversion,
    ])

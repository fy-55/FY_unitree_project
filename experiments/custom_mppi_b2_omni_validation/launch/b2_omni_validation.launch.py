#!/usr/bin/env python3
"""Launch the isolated ideal-response B2-style omnidirectional Nav2 plant."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    root = Path(__file__).resolve().parents[1]
    world = root / "worlds/b2_omni_world.world"
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    params_file = LaunchConfiguration("params_file")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch/gzserver.launch.py")
        ),
        launch_arguments={
            "world": str(world),
            "server_required": "true",
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--yaw", "0",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )
    base_to_scan = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_scan",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.30",
            "--yaw", "0",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "base_scan",
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / "launch/navigation_launch.py")
        ),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": "true",
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="Generated custom or official Nav2 parameter file",
            ),
            gazebo,
            map_to_odom,
            base_to_scan,
            map_server,
            map_lifecycle,
            # Let the transient-local map publisher activate before the
            # planner costmap subscribes. Starting both lifecycle managers at
            # the same instant occasionally left the static layer at its
            # default 5 m map until restart.
            TimerAction(period=2.0, actions=[navigation]),
        ]
    )

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_nav_control"))

    params_file = LaunchConfiguration("params_file")
    enable_motion = LaunchConfiguration("enable_motion")

    bridge = Node(
        package="g1_nav_control",
        executable="g1_velocity_bridge",
        name="g1_velocity_bridge",
        output="screen",
        parameters=[
            params_file,
            {
                "enable_motion": ParameterValue(
                    enable_motion, value_type=bool
                )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    package_share / "config" / "velocity_bridge.yaml"
                ),
                description="Absolute path to the G1 velocity bridge parameters",
            ),
            DeclareLaunchArgument(
                "enable_motion",
                default_value="false",
                description=(
                    "Actually publish Unitree API 7105 commands. Keep false "
                    "for every offline and first interface test."
                ),
            ),
            LogInfo(
                msg=[
                    "Starting /cmd_vel_safe + /scan -> "
                    "/api/sport/request bridge with enable_motion=",
                    enable_motion,
                    ". API output remains disabled when false.",
                ]
            ),
            bridge,
        ]
    )

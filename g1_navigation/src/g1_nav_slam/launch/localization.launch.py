import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_localization(context):
    map_name = LaunchConfiguration("map_name").perform(context)
    map_dir = Path(LaunchConfiguration("map_dir").perform(context)).expanduser()
    params_file = LaunchConfiguration("params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    if not re.fullmatch(r"[A-Za-z0-9_-]+", map_name):
        raise RuntimeError(
            "map_name can contain only letters, digits, underscores, and hyphens; "
            "do not add .yaml, .posegraph, or another extension"
        )

    map_base = map_dir / map_name
    required_files = [
        map_base.with_suffix(".pgm"),
        map_base.with_suffix(".yaml"),
        map_base.with_suffix(".posegraph"),
        map_base.with_suffix(".data"),
    ]
    missing_files = [path for path in required_files if not path.is_file()]
    if missing_files:
        missing_text = "\n  ".join(str(path) for path in missing_files)
        raise RuntimeError(
            "The selected map is incomplete. Missing files:\n  " + missing_text
        )

    localization_node = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            params_file,
            {
                "map_file_name": str(map_base),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    return [
        LogInfo(msg=f"Loading G1 SLAM localization map: {map_base}"),
        localization_node,
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_nav_slam"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_name",
                description="Saved map basename, for example lab_01 (no extension)",
            ),
            DeclareLaunchArgument(
                "map_dir",
                default_value=str(package_share / "maps"),
                description="Directory containing the four saved map files",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "localization.yaml"),
                description="Absolute path to the G1 localization parameters",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulator /clock instead of the physical clock",
            ),
            OpaqueFunction(function=_launch_localization),
        ]
    )

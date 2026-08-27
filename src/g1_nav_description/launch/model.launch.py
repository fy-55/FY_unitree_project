from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("g1_nav_description")
    )

    xacro_file = package_share / "urdf" / "g1_nav.urdf.xacro"

    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command(["xacro ", str(xacro_file)]),
        value_type=str,
    )

    use_sim_time_parameter = ParameterValue(
        use_sim_time,
        value_type=bool,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time_parameter,
            }
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(use_joint_state_gui),
        parameters=[
            {
                "use_sim_time": use_sim_time_parameter,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_joint_state_gui",
                default_value="false",
                description=(
                    "Start a fake joint-state GUI for model inspection. "
                    "Set false for simulation or the physical G1."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the /clock topic provided by a simulator.",
            ),
            # 根据URDF和关节状态计算机器人内部相对位姿。
            robot_state_publisher_node,
            # 离线检查模型时模拟真机关节状态。
            joint_state_publisher_gui_node,
        ]
    )

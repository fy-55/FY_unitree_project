from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = Path(get_package_share_directory("g1_nav_nav2"))
    nav2_bringup_share = Path(get_package_share_directory("nav2_bringup"))

    params_file = LaunchConfiguration("params_file")
    collision_monitor_params_file = LaunchConfiguration(
        "collision_monitor_params_file"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    # In ROS 2 Humble, parameters supplied while loading a composable
    # controller_server are filtered for that component. Its internally-created
    # local_costmap node therefore would otherwise fall back to Nav2 defaults.
    # Put the rewritten file on the container process command line as well, so
    # the internal local/global costmap nodes can resolve their own sections.
    container_params_file = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        },
        convert_types=True,
    )

    nav2_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_isolated",
        name="nav2_container",
        namespace="",
        output="screen",
        composable_node_descriptions=[],
        arguments=[
            "--ros-args",
            "--params-file",
            container_params_file,
            "--log-level",
            log_level,
        ],
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_bringup_share / "launch" / "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": "True",
            "container_name": "nav2_container",
            "use_respawn": "False",
            "log_level": log_level,
        }.items(),
    )

    collision_monitor = LifecycleNode(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        namespace="",
        output="screen",
        parameters=[
            collision_monitor_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    static_footprint_publisher = Node(
        package="g1_nav_nav2",
        executable="static_footprint_publisher",
        name="static_footprint_publisher",
        namespace="",
        output="screen",
        parameters=[
            collision_monitor_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    activate_collision_monitor = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=collision_monitor,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(collision_monitor),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    configure_collision_monitor = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(collision_monitor),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "nav2.yaml"),
                description="Absolute path to the G1 Nav2 parameter file",
            ),
            DeclareLaunchArgument(
                "collision_monitor_params_file",
                default_value=str(
                    package_share / "config" / "collision_monitor.yaml"
                ),
                description="Absolute path to the G1 collision monitor parameters",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulator /clock instead of the physical clock",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically configure and activate Nav2 lifecycle nodes",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Nav2 log level",
            ),
            LogInfo(
                msg=(
                    "Starting G1 Nav2 and Collision Monitor in no-motion mode. "
                    "Safe output is /cmd_vel_safe; no G1 motion bridge is launched."
                )
            ),
            nav2_container,
            nav2_navigation,
            static_footprint_publisher,
            collision_monitor,
            activate_collision_monitor,
            configure_collision_monitor,
        ]
    )

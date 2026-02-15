import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("LeKiwi", package_name="lekiwi_moveit").to_moveit_configs()
    # Load SRDF as raw XML text to avoid xacro-generated comment artifacts
    # that can break strict SRDF parsers in RViz/MoveIt.
    srdf_path = Path(get_package_share_directory("lekiwi_moveit")) / "config" / "LeKiwi.srdf"
    moveit_config.robot_description_semantic = {
        "robot_description_semantic": srdf_path.read_text(encoding="utf-8")
    }

    # Inlined from generate_move_group_launch() to add the /joint_states
    # remap required by the namespaced controller_manager (namespace: lekiwi).
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # The controller_manager runs in the "lekiwi" namespace, so
    # joint_state_broadcaster publishes to /lekiwi/joint_states.
    remappings = [("/joint_states", "/lekiwi/joint_states")]

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
            remappings=remappings,
            condition=UnlessCondition(LaunchConfiguration("debug")),
            additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        )
    )

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
            remappings=remappings,
            prefix=[
                f"gdb -x {moveit_config.package_path / 'launch' / 'gdb_settings.gdb'} --ex run --args"
            ],
            arguments=["--debug"],
            condition=IfCondition(LaunchConfiguration("debug")),
            additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        )
    )

    return ld

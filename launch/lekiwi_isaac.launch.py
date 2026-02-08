# Copyright 2025 Albert Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


lekiwi_share_dir = Path(get_package_share_directory("lekiwi"))
lekiwi_share_parent_dir = str(lekiwi_share_dir.parent)

ARGUMENTS = [
    DeclareLaunchArgument(
        "isaac_root",
        default_value="/home/andcav/isaacsim",
        description="Isaac Sim install root (contains python.sh).",
    ),
    DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Isaac Sim headless [true/false].",
    ),
    DeclareLaunchArgument(
        "renderer",
        default_value="RaytracedLighting",
        description="Isaac Sim renderer backend.",
    ),
    DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("lekiwi"), "urdf", "lekiwi.urdf"]
        ),
        description="Absolute path to LeKiwi URDF.",
    ),
    DeclareLaunchArgument("robot_name", default_value="lekiwi", description="Robot name label."),
    DeclareLaunchArgument("x", default_value="0.0", description="Spawn X [m]."),
    DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y [m]."),
    DeclareLaunchArgument("z", default_value="0.0", description="Spawn Z [m]."),
    DeclareLaunchArgument("roll", default_value="0.0", description="Spawn roll [rad]."),
    DeclareLaunchArgument("pitch", default_value="0.0", description="Spawn pitch [rad]."),
    DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw [rad]."),
    DeclareLaunchArgument(
        "fix_base",
        default_value="false",
        description="Fix base link [true/false].",
    ),
    DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="Start ros2_control (controller_manager + controllers).",
    ),
    DeclareLaunchArgument(
        "enable_wheel_velocity_bridge",
        default_value="true",
        description="Start wheel velocity bridge node for Isaac wheel control.",
    ),
    DeclareLaunchArgument(
        "enable_cmd_vel_to_wheel",
        default_value="true",
        description="Start cmd_vel to wheel velocity translator node.",
    ),
    DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/lekiwi/cmd_vel",
        description="Twist topic consumed by cmd_vel to wheel translator.",
    ),
    DeclareLaunchArgument(
        "cmd_vel_wheel_angles_deg",
        default_value="[60.0, 180.0, 300.0]",
        description="Wheel alphas in degrees for cmd_vel kinematics in joint order [joint7,joint8,joint9].",
    ),
    DeclareLaunchArgument(
        "cmd_vel_wheel_signs",
        default_value="[1.0, 1.0, 1.0]",
        description="Wheel direction signs for cmd_vel kinematics in joint order [joint7,joint8,joint9].",
    ),
    DeclareLaunchArgument(
        "cmd_vel_wheel_radius_m",
        default_value="0.05",
        description="Wheel radius in meters for cmd_vel kinematics.",
    ),
    DeclareLaunchArgument(
        "cmd_vel_robot_radius_m",
        default_value="0.125",
        description="Robot radius in meters for cmd_vel kinematics.",
    ),
    DeclareLaunchArgument(
        "cmd_vel_linear_deadband_m_s",
        default_value="0.01",
        description="Deadband for linear cmd_vel x/y components before wheel conversion.",
    ),
    DeclareLaunchArgument(
        "cmd_vel_angular_deadband_rad_s",
        default_value="0.05",
        description="Deadband for angular cmd_vel z component before wheel conversion.",
    ),
    DeclareLaunchArgument(
        "wheel_direct_velocity",
        default_value="",
        description="Direct wheel velocity in Isaac without ROS wheel topic. "
        "Format: '' (disabled), '3.0' (all wheels), or '3.0,3.0,3.0' "
        "(joint7,joint8,joint9) in rad/s.",
    ),
    DeclareLaunchArgument(
        "debug_disable_roller_collisions",
        default_value="false",
        description="Debug: disable collision shapes for omni3 roller links.",
    ),
    DeclareLaunchArgument(
        "debug_disable_rim_collisions",
        default_value="false",
        description="Debug: disable collision shapes for omni3 rim links.",
    ),
    DeclareLaunchArgument(
        "debug_disable_all_wheel_collisions",
        default_value="false",
        description="Debug: disable collision shapes for all wheel bodies.",
    ),
    DeclareLaunchArgument(
        "startup_brake_seconds",
        default_value="0.0",
        description="Duration [s] after Play where base/wheels are clamped to zero. Set > 0 to enable startup drift suppression.",
    ),
    DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Start RViz2 (off by default).",
    ),
    DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="true",
        description="Publish /joint_states (zeros) so robot_state_publisher can publish TF.",
    ),
    DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("lekiwi"), "rviz", "lekiwi_world_simple.rviz"]
        ),
        description="RViz2 config file.",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock if provided by Isaac Sim.",
    ),
    DeclareLaunchArgument(
        "controllers_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("lekiwi"), "config", "ros2_controllers.yaml"]
        ),
        description="ros2_control controllers YAML file.",
    ),
    DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use mock_components/GenericSystem in ros2_control description.",
    ),
    DeclareLaunchArgument(
        "ros_package_path",
        default_value=lekiwi_share_parent_dir,
        description="ROS_PACKAGE_PATH prefix for resolving package:// meshes.",
    ),
]


def generate_launch_description():
    isaac_root = LaunchConfiguration("isaac_root")
    headless = LaunchConfiguration("headless")
    renderer = LaunchConfiguration("renderer")
    urdf_path = LaunchConfiguration("urdf_path")
    robot_name = LaunchConfiguration("robot_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    fix_base = LaunchConfiguration("fix_base")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    enable_wheel_velocity_bridge = LaunchConfiguration("enable_wheel_velocity_bridge")
    enable_cmd_vel_to_wheel = LaunchConfiguration("enable_cmd_vel_to_wheel")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_wheel_angles_deg = LaunchConfiguration("cmd_vel_wheel_angles_deg")
    cmd_vel_wheel_signs = LaunchConfiguration("cmd_vel_wheel_signs")
    cmd_vel_wheel_radius_m = LaunchConfiguration("cmd_vel_wheel_radius_m")
    cmd_vel_robot_radius_m = LaunchConfiguration("cmd_vel_robot_radius_m")
    cmd_vel_linear_deadband_m_s = LaunchConfiguration("cmd_vel_linear_deadband_m_s")
    cmd_vel_angular_deadband_rad_s = LaunchConfiguration("cmd_vel_angular_deadband_rad_s")
    wheel_direct_velocity = LaunchConfiguration("wheel_direct_velocity")
    debug_disable_roller_collisions = LaunchConfiguration("debug_disable_roller_collisions")
    debug_disable_rim_collisions = LaunchConfiguration("debug_disable_rim_collisions")
    debug_disable_all_wheel_collisions = LaunchConfiguration("debug_disable_all_wheel_collisions")
    startup_brake_seconds = LaunchConfiguration("startup_brake_seconds")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controllers_file = LaunchConfiguration("controllers_file")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    ros_package_path = LaunchConfiguration("ros_package_path")

    ros_pkg_env = SetEnvironmentVariable(
        name="ROS_PACKAGE_PATH",
        value=[ros_package_path, ":", EnvironmentVariable("ROS_PACKAGE_PATH", default_value="")],
    )
    python_unbuffered_env = SetEnvironmentVariable(name="PYTHONUNBUFFERED", value="1")

    script_path = PathJoinSubstitution(
        [FindPackageShare("lekiwi"), "scripts", "lekiwi_isaac_sim.py"]
    )

    isaac_sim = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([isaac_root, "python.sh"]),
            script_path,
            "--urdf",
            urdf_path,
            "--headless",
            headless,
            "--renderer",
            renderer,
            "--robot-name",
            robot_name,
            "--x",
            x,
            "--y",
            y,
            "--z",
            z,
            "--roll",
            roll,
            "--pitch",
            pitch,
            "--yaw",
            yaw,
            "--fix-base",
            fix_base,
            "--wheel-direct-velocity",
            wheel_direct_velocity,
            "--debug-disable-roller-collisions",
            debug_disable_roller_collisions,
            "--debug-disable-rim-collisions",
            debug_disable_rim_collisions,
            "--debug-disable-all-wheel-collisions",
            debug_disable_all_wheel_collisions,
            "--startup-brake-seconds",
            startup_brake_seconds,
            "--ros-package-path",
            ros_package_path,
        ],
        output="screen",
    )

    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [FindPackageShare("lekiwi"), "config", "lekiwi.urdf.xacro"]
            ),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace="lekiwi",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
            controllers_file,
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    joint_command_mux_node = Node(
        package="lekiwi",
        executable="lekiwi_joint_command_mux.py",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_ros2_control),
    )

    wheel_velocity_bridge_node = Node(
        package="lekiwi",
        executable="lekiwi_wheel_velocity_bridge.py",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(enable_wheel_velocity_bridge),
    )

    cmd_vel_to_wheel_node = Node(
        package="lekiwi",
        executable="lekiwi_cmd_vel_to_wheel_velocity.py",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"cmd_vel_topic": cmd_vel_topic},
            {"wheel_angles_deg": cmd_vel_wheel_angles_deg},
            {"wheel_signs": cmd_vel_wheel_signs},
            {"wheel_radius_m": cmd_vel_wheel_radius_m},
            {"robot_radius_m": cmd_vel_robot_radius_m},
            {"linear_deadband_m_s": cmd_vel_linear_deadband_m_s},
            {"angular_deadband_rad_s": cmd_vel_angular_deadband_rad_s},
        ],
        condition=IfCondition(enable_cmd_vel_to_wheel),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/lekiwi/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    arm_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_position_controller",
            "--controller-manager",
            "/lekiwi/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "--controller-manager",
            "/lekiwi/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager",
            "/lekiwi/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
        condition=IfCondition(
            PythonExpression([
                "'",
                use_joint_state_publisher,
                "' == 'true' and '",
                use_ros2_control,
                "' == 'false'",
            ])
        ),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ros_pkg_env)
    ld.add_action(python_unbuffered_env)
    ld.add_action(isaac_sim)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(control_node)
    ld.add_action(joint_command_mux_node)
    ld.add_action(wheel_velocity_bridge_node)
    ld.add_action(cmd_vel_to_wheel_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_position_controller_spawner)
    ld.add_action(gripper_position_controller_spawner)
    ld.add_action(wheel_velocity_controller_spawner)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    return ld

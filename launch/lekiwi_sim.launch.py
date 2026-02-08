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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use mock_components/GenericSystem instead of real hardware.",
    ),
    DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2.",
    ),
    DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("lekiwi"), "rviz", "lekiwi.rviz"]
        ),
        description="RViz2 config file.",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock (only if a simulator provides it).",
    ),
]


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
            PathJoinSubstitution(
                [FindPackageShare("lekiwi"), "config", "ros2_controllers.yaml"]
            ),
        ],
        remappings=[
            ("~/odom", "/odom"),
        ],
        output="screen",
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    return ld

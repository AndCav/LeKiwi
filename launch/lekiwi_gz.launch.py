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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


ARGUMENTS = [
    DeclareLaunchArgument(
        "gz_args",
        default_value="-r -v 4 empty.sdf",
        description="Arguments passed to `gz sim` (world, -r, -g, etc).",
    ),
    DeclareLaunchArgument(
        "world_name",
        default_value="empty",
        description="Gazebo world name used by the spawn service (/world/<name>/create).",
    ),
    DeclareLaunchArgument("robot_name", default_value="lekiwi", description="Spawned entity name."),
    DeclareLaunchArgument("x", default_value="0.0", description="Spawn X [m]."),
    DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y [m]."),
    DeclareLaunchArgument("z", default_value="0.2", description="Spawn Z [m]."),
    DeclareLaunchArgument("roll", default_value="0.0", description="Spawn roll [rad]."),
    DeclareLaunchArgument("pitch", default_value="0.0", description="Spawn pitch [rad]."),
    DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw [rad]."),
    DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="Start ros2_control (controller_manager + controllers). With fake hardware it won't drive Gazebo physics.",
    ),
    DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Start RViz2.",
    ),
    DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="true",
        description="Publish /joint_states (zeros) so robot_state_publisher can publish TF.",
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
        default_value="true",
        description="Use /clock (bridged from Gazebo).",
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
]


def generate_launch_description():
    gz_args = LaunchConfiguration("gz_args")
    world_name = LaunchConfiguration("world_name")
    robot_name = LaunchConfiguration("robot_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controllers_file = LaunchConfiguration("controllers_file")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Gazebo resolves `model://<name>/...` by searching for a `<name>` folder under
    # each entry in GZ_SIM_RESOURCE_PATH. When LeKiwi meshes are referenced as:
    #   model://lekiwi/urdf/meshes/...
    # we need to add the parent of the package share directory (i.e. the `share/`
    # directory that contains the `lekiwi/` folder), not the package share itself.
    lekiwi_share_dir = Path(get_package_share_directory("lekiwi"))
    lekiwi_share_parent_dir = str(lekiwi_share_dir.parent)

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ":",
            lekiwi_share_parent_dir,
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
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
        parameters=[
            {"use_sim_time": use_sim_time},
            controllers_file,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("~/odom", "/odom"),
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
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
            PythonExpression(
                [
                    "'",
                    use_joint_state_publisher,
                    "' == 'true' and '",
                    use_ros2_control,
                    "' == 'false'",
                ]
            )
        ),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": world_name,
                "string": robot_description,
                "name": robot_name,
                "allow_renaming": False,
                "x": x,
                "y": y,
                "z": z,
                "R": roll,
                "P": pitch,
                "Y": yaw,
            }
        ],
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
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn)
    ld.add_action(rviz_node)
    return ld

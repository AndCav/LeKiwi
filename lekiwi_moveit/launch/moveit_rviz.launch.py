from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("LeKiwi", package_name="lekiwi_moveit").to_moveit_configs()
    # Load SRDF as raw XML text to avoid xacro-generated comment artifacts
    # that can break strict SRDF parsers in RViz/MoveIt.
    srdf_path = Path(get_package_share_directory("lekiwi_moveit")) / "config" / "LeKiwi.srdf"
    moveit_config.robot_description_semantic = {
        "robot_description_semantic": srdf_path.read_text(encoding="utf-8")
    }
    return generate_moveit_rviz_launch(moveit_config)

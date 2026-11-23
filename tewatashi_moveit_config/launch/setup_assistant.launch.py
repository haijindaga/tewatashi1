"""Generated-style launch file, as in my_robot_cell_moveit_config (official UR tutorial)."""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("tewatashi_cell", package_name="tewatashi_moveit_config")
        # As in ur_moveit_config: MoveIt does not switch controllers itself.
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False
        )
        .to_moveit_configs()
    )
    return generate_setup_assistant_launch(moveit_config)

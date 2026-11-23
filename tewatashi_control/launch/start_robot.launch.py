"""Start the UR driver (mock hardware, URSim or the real robot) for the cell.

Thin wrapper around ur_robot_driver/ur_control.launch.py, following
my_robot_cell_control/launch/start_robot.launch.py of the official UR
tutorial.

Examples:
  # Mock hardware (default, no robot needed)
  ros2 launch tewatashi_control start_robot.launch.py
  # URSim (Docker)
  ros2 launch tewatashi_control start_robot.launch.py use_fake_hardware:=false robot_ip:=192.168.56.101
  # Real robot
  ros2 launch tewatashi_control start_robot.launch.py use_fake_hardware:=false robot_ip:=<ROBOT_IP>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur7e",
            choices=["ur7e", "ur5e"],
            description="UR model. ur7e and ur5e share the same kinematics.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            # Project choice: default to mock hardware so that forgetting the
            # argument never connects to a real robot.
            default_value="true",
            description="true: ros2_control mock hardware; false: URSim or the real robot.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="yyy.yyy.yyy.yyy",
            description="Robot IP. Unused with mock hardware. URSim: 192.168.56.101.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("tewatashi_control"), "config", "ur7e", "default_kinematics.yaml"]
            ),
            description="Kinematics calibration. Replace with the file extracted by "
            "ur_calibration from the real robot before precise grasping.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Headless mode of the UR driver.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Driver's own RViz. Off: use tewatashi_moveit_config's RViz instead.",
        ),
    ]

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch"]), "/ur_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "tf_prefix": "",
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "description_package": "tewatashi_control",
            "description_file": "tewatashi_cell_controlled.urdf.xacro",
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            # scaled_joint_trajectory_controller works on mock hardware with
            # ur_robot_driver 2.13 and is what MoveIt is configured to use.
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "headless_mode": LaunchConfiguration("headless_mode"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [base_launch])

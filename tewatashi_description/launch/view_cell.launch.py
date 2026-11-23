"""Show the cell in RViz with sliders for the joints (no robot driver).

Based on my_robot_cell_description/launch/view_robot.launch.py of the
official UR tutorial.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    description_package = FindPackageShare("tewatashi_description")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "tewatashi_cell.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution([description_package, "rviz", "view_cell.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", description_file, " ", "ur_type:=", ur_type]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type", default_value="ur7e", choices=["ur7e", "ur5e"],
                description="UR model. ur7e and ur5e share the same kinematics.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )

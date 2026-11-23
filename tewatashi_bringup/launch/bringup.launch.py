"""Start everything with one command.

  ros2 launch tewatashi_bringup bringup.launch.py                 # mock hardware
  ros2 launch tewatashi_bringup bringup.launch.py \
      use_fake_hardware:=false robot_ip:=<IP> kinematics_params_file:=<yaml>

Started:
  driver (ur_robot_driver via tewatashi_control), move_group, RViz,
  task node (YASMIN), yasmin_viewer, rosbridge, and a static web server for
  the operator panel at http://127.0.0.1:8080 .
All network services listen on 127.0.0.1 only.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control = get_package_share_directory("tewatashi_control")
    moveit = get_package_share_directory("tewatashi_moveit_config")
    task = get_package_share_directory("tewatashi_task")

    use_fake = LaunchConfiguration("use_fake_hardware")
    panel_port = LaunchConfiguration("panel_port")

    args = [
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="true: mock hardware (default), false: real robot"),
        DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=os.path.join(control, "config", "ur7e", "default_kinematics.yaml"),
            description="Calibration extracted with ur_calibration (real robot)"),
        DeclareLaunchArgument("panel_port", default_value="8080"),
        DeclareLaunchArgument("open_browser", default_value="true"),
    ]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control, "launch", "start_robot.launch.py")),
        launch_arguments={
            "use_fake_hardware": use_fake,
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "launch_rviz": "false",
        }.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit, "launch", "move_group.launch.py")))
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit, "launch", "moveit_rviz.launch.py")))

    task_node = Node(
        package="tewatashi_task",
        executable="task_node",
        name="tewatashi_task",
        output="screen",
        parameters=[
            os.path.join(task, "config", "task.yaml"),
            {"use_fake_hardware": ParameterValue(use_fake, value_type=bool)},
        ],
    )
    viewer = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        parameters=[{"host": "127.0.0.1", "port": 5000}],
    )
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        parameters=[{"address": "127.0.0.1", "port": 9090}],
    )
    panel_server = ExecuteProcess(
        cmd=["python3", "-m", "http.server", panel_port,
             "--bind", "127.0.0.1", "--directory", os.path.join(task, "web")],
    )
    browser = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=["xdg-open", ["http://127.0.0.1:", panel_port]],
            condition=IfCondition(LaunchConfiguration("open_browser")),
        )],
    )

    return LaunchDescription(
        args + [driver, move_group, rviz, task_node, viewer, rosbridge, panel_server, browser]
    )

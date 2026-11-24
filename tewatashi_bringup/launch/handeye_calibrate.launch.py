"""Eye-in-hand calibration of the wrist D435 (easy_handeye2 + aruco_opencv).

  ros2 launch tewatashi_bringup handeye_calibrate.launch.py \
      use_fake_hardware:=false robot_ip:=<IP> kinematics_params_file:=<yaml>

Started: UR driver (joint states / TF only; the arm is moved by hand with
freedrive, not by ROS), RealSense driver under the name "calib_cam", the ArUco
board tracker, and the easy_handeye2 sampling GUI.

Why "calib_cam": the URDF already fixes a provisional camera_link to tool0.
The calibration needs the camera's own TF tree unattached, so the driver
publishes calib_cam_link -> ... -> calib_cam_color_optical_frame separately.
easy_handeye2 then computes tool0 -> calib_cam_link, which is directly the URDF
camera mount (tools/handeye_to_urdf.py prints the arguments).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CALIB_NAME = "tewatashi_eih"
CAMERA_NAME = "calib_cam"
BOARD_NAME = "calib"  # name in config/handeye_board.yaml -> TF frame board_calib


def generate_launch_description():
    control = get_package_share_directory("tewatashi_control")
    bringup = get_package_share_directory("tewatashi_bringup")
    realsense = get_package_share_directory("realsense2_camera")
    handeye = get_package_share_directory("easy_handeye2")

    args = [
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=os.path.join(control, "config", "ur7e", "default_kinematics.yaml")),
    ]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control, "launch", "start_robot.launch.py")),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "launch_rviz": "false",
        }.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense, "launch", "rs_launch.py")),
        launch_arguments={
            "camera_name": CAMERA_NAME,
            "camera_namespace": "camera",
            "enable_depth": "false",  # colour only for marker detection
        }.items(),
    )
    tracker = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[
            os.path.join(bringup, "config", "aruco_handeye.yaml"),
            {"board_descriptions_path": os.path.join(bringup, "config", "handeye_board.yaml")},
        ],
    )
    calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(handeye, "launch", "calibrate.launch.py")),
        launch_arguments={
            "name": CALIB_NAME,
            "calibration_type": "eye_in_hand",
            "robot_base_frame": "base_link",
            "robot_effector_frame": "tool0",
            "tracking_base_frame": f"{CAMERA_NAME}_link",
            "tracking_marker_frame": f"board_{BOARD_NAME}",
        }.items(),
    )
    return LaunchDescription(args + [driver, camera, tracker, calibrate])

#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    pkg_share = get_package_share_directory("my_arm_control")

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context).strip().lower()

    if robot_model == "puma":
        exe = "send_pose_trajectory_puma_simple_exe"
        params_file = os.path.join(pkg_share, "config", "pose_puma.yaml")
        msg = "PUMA robot arm has Spherical-wrist"

    elif robot_model == "ur5e":
        exe = "send_pose_trajectory_ur5e_exe"
        params_file = os.path.join(pkg_share, "config", "pose_ur5e.yaml")
        msg = "UR5e robot arm has not Spherical-wrist"

    else:
        # Safe fallback
        exe = "send_pose_trajectory_puma_limits_exe"
        params_file = os.path.join(pkg_share, "config", "pose_puma.yaml")
        msg = (
            f"Unknown robot_model='{robot_model}'. "
            "Defaulting to puma_simple behavior (spherical-wrist)."
        )

    node = Node(
        package="my_arm_control",
        executable=exe,
        name="send_pose_trajectory",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": (use_sim_time.lower() == "true")},
        ],
    )

    return [
        LogInfo(msg=msg),
        LogInfo(msg=f"Using params file: {os.path.basename(params_file)}"),
        LogInfo(msg=f"Launching executable: {exe}"),
        node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="puma",
            description="Robot model selector: 'puma' or 'ur5e'",
        ),
        OpaqueFunction(function=_make_nodes),
    ])

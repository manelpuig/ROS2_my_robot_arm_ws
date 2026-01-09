#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("my_arm_control")
    params_file = os.path.join(pkg_share, "config", "tool_pose.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),

        Node(
            package="my_arm_control",
            executable="move_tool_to_pose_exe",   # <- posa aquí el teu executable real
            name="move_tool_to_pose",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])

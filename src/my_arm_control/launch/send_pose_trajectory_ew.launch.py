#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("my_arm_control")
    params_file = os.path.join(pkg_share, "config", "ik_tool_pose.yaml")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

    node = Node(
        package="my_arm_control",
        executable="move_tool_to_pose_exe",
        name="move_tool_to_pose",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([use_sim_time_arg, node])

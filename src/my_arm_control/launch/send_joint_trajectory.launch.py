#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("my_arm_control")
    params_file = os.path.join(pkg_share, "config", "poses.yaml")

    node = Node(
        package="my_arm_control",
        executable="send_joint_trajectory",
        name="send_joint_trajectory",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([node])

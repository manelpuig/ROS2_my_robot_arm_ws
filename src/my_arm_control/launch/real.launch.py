#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("my_arm_description")
    ctrl_share = get_package_share_directory("my_arm_control")

    xacro_path = os.path.join(desc_share, "urdf", "my_arm.urdf.xacro")
    params_path = os.path.join(ctrl_share, "config", "arm_params.yaml")

    robot_description = {"robot_description": Command(["xacro ", xacro_path])}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    serial_bridge = Node(
        package="my_arm_control",
        executable="serial_bridge",
        parameters=[params_path],
        output="screen",
    )

    joint_state_estimator = Node(
        package="my_arm_control",
        executable="joint_state_estimator",
        parameters=[params_path],
        output="screen",
    )

    return LaunchDescription([rsp, serial_bridge, joint_state_estimator])

#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("my_arm_description")
    xacro_path = os.path.join(pkg_share, "urdf", "my_arm.urdf.xacro")
    rviz_path = os.path.join(pkg_share, "rviz", "my_arm.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]),
        value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        output="screen",
    )

    return LaunchDescription([rsp, jsp_gui, rviz])

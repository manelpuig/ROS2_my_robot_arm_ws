#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("my_arm_description")
    gz_share = get_package_share_directory("my_arm_gz")

    xacro_path = os.path.join(desc_share, "urdf", "my_arm.urdf.xacro")
    world_path = os.path.join(gz_share, "worlds", "empty.sdf")
    rviz_path = os.path.join(desc_share, "rviz", "my_arm.rviz")
    controllers_yaml = os.path.join(gz_share, "config", "gz_controllers.yaml")

    robot_description = {"robot_description": Command(["xacro ", xacro_path])}

    # Gazebo Sim (ros_gz_sim provides gz_sim.launch.py; here we call gz directly for clarity)
    # Alternative: ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=...
    gz = ExecuteProcess(
        cmd=["ros2", "launch", "ros_gz_sim", "gz_sim.launch.py", f"gz_args:={world_path} -r"],
        output="screen",
    )

    # Robot State Publisher for TF in ROS side (useful for RViz)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Spawn entity into Gazebo from robot_description
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_arm",
            "-topic", "robot_description",
            "-z", "0.05",
        ],
        output="screen",
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        output="screen",
    )

    # Spawn controllers (controller_manager is created by gz_ros2_control plugin inside Gazebo)
    # Delay a bit to let the entity + controller_manager come up.
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[controllers_yaml],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[controllers_yaml],
        output="screen",
    )

    return LaunchDescription([
        gz,
        rsp,
        spawn,
        rviz,
        TimerAction(period=3.0, actions=[spawner_jsb]),
        TimerAction(period=4.0, actions=[spawner_arm]),
    ])

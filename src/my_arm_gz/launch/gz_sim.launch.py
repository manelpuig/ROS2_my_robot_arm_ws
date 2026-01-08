#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("my_arm_description")
    gz_share = get_package_share_directory("my_arm_gz")

    xacro_path = os.path.join(desc_share, "urdf", "my_arm.urdf.xacro")
    world_path = os.path.join(gz_share, "worlds", "empty.sdf")
    rviz_path = os.path.join(desc_share, "rviz", "my_arm.rviz")
    controllers_yaml = os.path.join(gz_share, "config", "gz_controllers.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_path,
            " ros2_control_params:=",
            controllers_yaml,
        ]),
        value_type=str,
    )

    gz = ExecuteProcess(
        cmd=["ros2", "launch", "ros_gz_sim", "gz_sim.launch.py", f"gz_args:={world_path} -r"],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "my_arm", "-topic", "robot_description", "-z", "0.05"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        gz,
        rsp,
        spawn,
        rviz,
        TimerAction(period=3.0, actions=[spawner_jsb]),
        TimerAction(period=4.0, actions=[spawner_arm]),
    ])

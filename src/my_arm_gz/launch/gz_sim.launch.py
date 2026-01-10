#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("my_arm_description")
    gz_share = get_package_share_directory("my_arm_gz")

    default_model = "my_arm.urdf.xacro"
    world_path = os.path.join(gz_share, "worlds", "empty.sdf")
    rviz_path = os.path.join(desc_share, "rviz", "my_arm.rviz")
    controllers_yaml = os.path.join(gz_share, "config", "gz_controllers.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    declare_model = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Xacro file in my_arm_description/urdf",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")

    xacro_path = PathJoinSubstitution([
        TextSubstitution(text=desc_share),
        "urdf",
        model,
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            xacro_path, " ",
            "ros2_control_params:=", controllers_yaml,
        ]),
        value_type=str,
    )

    # Gazebo Sim first
    gz = ExecuteProcess(
        cmd=[
            "ros2", "launch", "ros_gz_sim", "gz_sim.launch.py",
            f"gz_args:={world_path} -r",
        ],
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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model,

        # Start Gazebo first
        gz,

        # Publish description immediately (TF tree exists early)
        rsp,

        # Spawn robot into Gazebo
        TimerAction(period=1.0, actions=[spawn]),

        # Start controllers after spawn
        TimerAction(period=3.0, actions=[spawner_jsb]),
        TimerAction(period=4.0, actions=[spawner_arm]),

        # Start RViz LAST (avoid early TF/clock issues)
        TimerAction(period=6.0, actions=[rviz]),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("my_arm_description")

    # -----------------------------
    # Launch arguments
    # -----------------------------
    declare_arm = DeclareLaunchArgument(
        "arm",
        default_value="my_arm_puma.urdf.xacro",
        description="URDF/Xacro file inside my_arm_description/urdf"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time (true for Gazebo / simulation)"
    )

    arm_file = LaunchConfiguration("arm")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # -----------------------------
    # Robot description from xacro
    # -----------------------------
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(pkg_share, "urdf", ""),
            arm_file
        ]),
        value_type=str
    )

    rviz_path = os.path.join(pkg_share, "rviz", "my_arm.rviz")

    # -----------------------------
    # Nodes
    # -----------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    return LaunchDescription([
        declare_arm,
        declare_use_sim_time,
        rsp,
        jsp_gui,
        rviz,
    ])

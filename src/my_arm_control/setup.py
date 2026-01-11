from setuptools import setup
from glob import glob
import os

package_name = "my_arm_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # Required for ROS 2 package discovery
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),

        ("share/" + package_name, ["package.xml"]),

        # Launch files
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),

        # YAML config files
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@todo.todo",
    description="ROS 2 control nodes for a 6-DOF arm (trajectory, serial, estimation).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "serial_bridge = my_arm_control.serial_bridge:main",
            "joint_state_estimator = my_arm_control.joint_state_estimator:main",
            "send_joint_trajectory_exe = my_arm_control.send_joint_trajectory:main",
            "send_pose_trajectory_ur5e_exe = my_arm_control.send_pose_trajectory_ur5e:main",
            "send_pose_trajectory_puma_exe = my_arm_control.send_pose_trajectory_puma:main",
        ],
    },
)

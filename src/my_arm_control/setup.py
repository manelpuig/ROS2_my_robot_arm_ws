from setuptools import setup

package_name = "my_arm_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/real.launch.py"]),
        ("share/" + package_name + "/config", ["config/arm_params.yaml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@todo.todo",
    description="Simple ROS2 control nodes for a 6DOF servo arm over USB serial.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "serial_bridge = my_arm_control.serial_bridge:main",
            "joint_state_estimator = my_arm_control.joint_state_estimator:main",
        ],
    },
)

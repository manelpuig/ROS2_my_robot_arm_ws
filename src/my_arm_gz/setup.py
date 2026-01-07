from setuptools import setup

package_name = "my_arm_gz"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/gz_sim.launch.py"]),
        ("share/" + package_name + "/worlds", ["worlds/empty.sdf"]),
        ("share/" + package_name + "/config", ["config/gz_controllers.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@todo.todo",
    description="Gazebo Sim bringup for my_arm.",
    license="Apache-2.0",
)

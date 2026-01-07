# Robot model

Once you have created a model you can:
- Review the arm model on RVIZ2:
````shell
ros2 launch my_arm_description display.launch.py
````
![](./Images/my_arm_rviz.png)

- Bringup the robot arm in Gazebo sim:
````shell
ros2 launch my_arm_gz gz_sim.launch.py
````

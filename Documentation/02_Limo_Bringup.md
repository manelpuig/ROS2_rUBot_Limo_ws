# **2. LIMO Bringup**

This repository is used for LIMO robot simulation in ROS2

References:

- Mastering ROS 2 with LIMO-Robot: https://www.robotigniteacademy.com/courses/227
- Class n.182: Learn ROS 2 with LIMO Robot: https://www.robotigniteacademy.com/rosjects/851570/
- bitbucket: https://bitbucket.org/theconstructcore/limo_robot/src/main/
- https://github.com/agilexrobotics/limo_ros2/tree/humble
- https://docs.ros.org/en/humble/Installation.html
- https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20Ros2%20Foxy%20user%20manual(EN).md

## **2.1. Bringup and control in Virtual environment**

To bringup the Limo robot in simulation environment:
- In a new terminal
````shell
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
````

To control using keyboard:
````shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
````
or The terminal:
````shell
rostopic pub -r 1 /cmd_vel geometry_msgs/Twist '[1, 0, 0]' '[0, 0, 1]'
````

## **2.2. Bringup and control of real Limo robot**

To bringup HW, in a new terminal:
````shell
ros2 launch limo_bringup limo_start.launch.py 
rviz2
````

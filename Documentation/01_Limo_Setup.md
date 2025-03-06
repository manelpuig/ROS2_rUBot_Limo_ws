# **1. LIMO robot Setup for ROS2 Humble**

This repository is used for LIMO robot simulation in ROS Noetic

References:

- https://github.com/agilexrobotics/limo_ros2/tree/humble
- https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20Ros2%20Foxy%20user%20manual(EN).md
- bitbucket: https://bitbucket.org/theconstructcore/limo_robot/src/main/

## **1.1. Simulation LIMO robot**

We have created a "ROS2_Limo_ws" github repository to fork on your github account and clone to your ROS environment (i.e. TheConstruct environment)

- clone the "ROS2_limo_ws" repository on Home ROS environment:
````shell
git clone https://github.com/your_username/ROS2_Limo_ws.git
````
- Build
````shell
cd ..
colcon build
source install/setup.bash
````
- Add in .bashrc the lines:
````shell
source /opt/ros/humble/setup.bash
source /home/user/ROS2_Limo_ws/install/setup.bash
cd /home/user/ROS2_Limo_ws
````
## **1.2. Real LIMO robot**

The real LIMO robot comes with a Jetson-Nano computer onboard with Ubuntu22 SO, ROS1 Noetic and  ROS2 Foxy installed. 

### **1.2.1. Setup on Ubuntu Desktop**

We can take the created repositories in limo robot for RO2 Foxy:

````shell
cd /home/agilex/limo_ros2_ws
````
You can simply source the .bashrc and work inside the repository.

### **1.2.2. Setup on Docker**

The available repositories in real LIMO robot Jetson Nano computer onboard are on ROS1 Noetic and ROS2 Foxy. If you want to work on other ROS2 distributions (like ROS2 Humble) the best method is to use Docker images:
- The process is describerd for ROS2 Humble in: https://hub.docker.com/r/theconstructai/limo

Careful!: add user to docker group: (https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)


# **2. LIMO Navigation**

This tutorial is using a simulation, but the process would be the same for a real Limo

References:
- https://limo-agx.github.io
- https://bitbucket.org/theconstructcore/limo_robot/src/main/

## **2.1. GMapping**

For navigation to be useful, the robot needs a map of the environment around it. It will use that map for high-level planning.

### **Creating a Map**

For navigation to be useful, the robot needs a map of the environment around it.

- Bringup the robot
- Launch the gmapping_node to create that map:
````shell
roslaunch limo_navigation limo_gmapping.launch
````
- To monitor the mapping process, you can run RVIZ
````shell
roslaunch limo_viz view_navigation_gmapping.launch
````
- Now you need to drive the robot around the environment. 
- Once your map looks complete, you will want to save it for future use
````shell
cd ~/rUBot_limo_ws/src/limo_sim/limo_navigation/maps
rosrun map_server map_saver
````
- Once the robot has a map of the environment, it can navigate around it
````shell
roslaunch limo_navigation limo_navigation_amcl_diff.launch
````
> verify the created map file in "map_file” parameter
> Close the slam node first
> Maintain opened the view_navigation_gmapping.launch

## **2.2. RTABMap**

We are going to use RTABMap to create a 3D map.

### **Creating a Map**

````shell
roslaunch limo_navigation limo_rtabmap.launch
````
- To monitor the mapping process, you can run RVIZ
````shell
roslaunch limo_viz view_navigation_rtabmap.launch
````
- Now you need to drive the robot around the environment. 
- Once your map looks complete, you will want to save it for future use
````shell
cd ~/rUBot_limo_ws/src/limo_sim/limo_navigation/maps
rosrun map_server map_saver -f rtab_map
````
- Once the robot has a map of the environment, it can navigate around it
````shell
roslaunch limo_navigation limo_navigation_rtabmap_diff.launch
````
> verify the created map file in "map_file” parameter
> Close the slam node first
> Maintain opened the view_navigation_gmapping.launch

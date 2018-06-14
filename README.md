# cobot_visualisation
The cobot_visualisation package is part of the cobot safety project. This package will calculate 

# Required packages:
universal_robot: http://wiki.ros.org/universal_robot <br/>
moveit_visual_tools: https://github.com/ros-planning/moveit_visual_tools

# Running a demo:
```
roslaunch cobot_visualisation ur5_demo
rosrun cobot_visualisation object_publisher_node
rosrun cobot_visualisation distance_node
rosrun cobot_visualisation visualisation_node
```
# Nodes used for normal operation:
```
rosrun cobot_visualisation distance_node
rosrun cobot_visualisation visualisation_node
```

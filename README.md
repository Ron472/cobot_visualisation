# cobot_visualisation
![banner](https://github.com/Ron472/cobot_visualisation/blob/master/images/banner.png)
The cobot_visualisation package is part of the cobot safety project ([Project video](https://www.youtube.com/watch?v=YmooYBySHqc)). This package is programmed using C++ and will calculate the closest distance between objects and the UR5 robot arm. A stop signal will be sent to the planner node when an object gets too close to the Robot. The package also adds the object and a workcell to the planning scene of MoveIt. The motion planner will try to plan around these objects. The distance between the closest object and the robot will also be visualised in Rviz.<br/>
<br/>
**Cobot Safety Project Packages:**<br/>
UR5CobotController: https://github.com/NielsPeulen/UR5CobotControler<br/>
OjbectDetection: https://github.com/StefanCals/ObjectDetection<br/>
cobot_visualisation (This package)


# Required packages:
The following pacakges are essential for running the cobot_visualisation package:<br/>
universal_robot: http://wiki.ros.org/universal_robot <br/>
moveit_visual_tools: https://github.com/ros-planning/moveit_visual_tools<br/>
<br/>
**Install for Ubuntu Debian:**
```
sudo apt-get update
sudo apt-get install ros-kinetic-universal-robot
sudo apt-get install ros-kinetic-moveit-visual-tools
```
# Installing cobot_visualisation
Copy the cobot_visualisation folder to your catkin_ws folder. Do catkin_make afterwards to compile the C++ code.
```
cd catkin_ws
catkin_make
```
# Running a demo (simulation):
With "simulated" object detection and a static robot. The object_publisher_node will provide objects for the distance_node. Run the following commands in seperate terminals:
```
roslaunch cobot_visualisation ur5_demo
rosrun cobot_visualisation object_publisher_node
rosrun cobot_visualisation distance_node
rosrun cobot_visualisation visualisation_node
```
# Nodes used for normal operation:
Requires that the [UR5CobotController package](https://github.com/NielsPeulen/UR5CobotControler) is already running. The [object detection package](https://github.com/StefanCals/ObjectDetection) will provide data for the distance_node. Run the following commands in seperate terminals:
```
rosrun cobot_visualisation distance_node
rosrun cobot_visualisation visualisation_node
```

# Documentation
More information and documentation about the cobot_visualisation package can be found on the [wiki](https://github.com/Ron472/cobot_visualisation/wiki).<br/>

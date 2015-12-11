Group 2 - Final Project
=========================



##About##

A ROS project developed for Northwestern ME495 - Embedded Systems in Robotics. This *baxter_two* package is used to control Baxter, a robot created by [Rethink Robotics], allowing Baxter to recognize several objects placed on a table, pick up the objects, and move them to a specified goal location. 

To launch (in a ROS workspace):

```bash
cd ~/baxter_ws/
. baxter.sh
rosrun baxter_tools enable_robot.py -e
roslaunch baxter_two baxter_two.launch
```
To run this demo on the Baxter simulation, use `roslaunch baxter_gazebo baxter_world.launch` to start up the simulator, and replace `. baxter.sh` with `. baxter.sh sim`.

**NOTE:** Prerequisites to set up workspace are listed as follows:

1.	Install the [Baxter SDK]
2.	Install [MoveIt!]
3.	Download [moveit_python] by Michael Ferguson
4.	Install Baxter Simulator (if running the simulation) **

**This must use version 1.1 of the simulator

##Package Contents##

Launch file:
-`baxter_two.launch`
	-Launches joint trajectory server, MoveIt! with rviz, baxter_img, and baxter_pnp.

nodes:
-baxter_img
	-Subscribes to the `/cameras/right_hand_camera/image` topic, calculates cube locations using OpenCV tools, and publishes this information
-baxter_pnp
	-Subscribes to the topic published to by *baxter_img*, then utilizes `PlanningSceneInterface` and `MoveGroupInterface` from *moveit_python* ** to create Baxter's world including the detected objects and plan/execute paths. 
	-Uses `baxter_interface` for gripper control


**Using these bindings, other capabilities include:
		-executing planned paths in both joint space and cartesian space
		-populating planning environment with collision objects
		-allows velocity scaling



##Overview##

This package utilizes tools offered in several different existing packages. The primary package used in Baxter's pick and place movements is the [MoveIt!] motion planning framework, paired with the [moveit_python] Python bindings created by Michael Ferguson. Additionally, [OpenCV] was used to locate the objects using one of Baxter's built-in cameras. This pick-and-place technique utilizes the built-in camera on Baxter's right arm to search for green cubes in the task space. Large green cubes are stacked on top of one another in the goal space, while small green cubes are simply placed in the goal space. Positions of objects can be dynamically repositioned and reoriented, and Baxter will continue to pick and place these object until there are no more object in the task space. 

##Important Nodes and Topics##

There are two nodes that are run in this package. The first node, `baxter_img`, subscribes to the `/cameras/right_hand_camera/image` topic, calculates cube locations using OpenCV tools, and publishes a `PoseArray()` message via the `DPos` topic to the second node, `baxter_pnp`. `baxter_pnp` then subscribes to the `DPos` published to by `baxter_img`. Then, it utilizes `PlanningSceneInterface` and `MoveGroupInterface` from *moveit_python* to create Baxter's world including the detected objects and plan/execute paths. 


##Future Work##

* One potential area of improvement for this package is in the camera calibration. currently, variations in surrounding lighting affect the object detection success. Using more finely-tuned methods of calibrating the camera will minimize the lighting issue. 

* Another area is in creating a Graphical User Interface that allows the user to click an object in the virtual world created by Baxter, and pick and place that particular item. In the current state, Baxter will pick up the object that it detects to be closest to itself. Currently, a GUI is under development that utilizes Gazebo pugin functionality. This GUI, when complete, would allow the user to choose an object presented in the Gazebo world, and click a button to execute the pick-and-place action.





[Rethink Robotics]: http://www.rethinkrobotics.com/baxter/
[MoveIt!]: https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial#tutorial
[Baxter setup instructions]: http://sdk.rethinkrobotics.com/wiki/Getting_Started
[here]: http://sdk.rethinkrobotics.com/wiki/Simulator_Installation
[moveit_python]: https://github.com/mikeferguson/moveit_python
[this tutorial]: https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial#tutorial
[OpenCV]: http://opencv.org/
[these instructions]: http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html

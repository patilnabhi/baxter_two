## About ##

A ROS project as part of ME495 - Embedded Systems in Robotics course at Northwestern University (NU). 

To launch (in a ROS workspace), type in terminal:
```
cd ~/baxter_ws/
. baxter.sh
rosrun baxter_tools enable_robot.py -e
roslaunch baxter_two baxter_two.launch
```

## Goal of Project ##

**Use baxter to pick and place objects from one location to another**

## Tutorial ##

The following sections describe contents of this package and how to use them:

### 1. Prerequisites ###

The following packages need to be installed for `baxter_two` package to work: 

1. [Baxter SDK] - to be installed in baxter_ws/src directory
2. [MoveIt!]
3. [moveit_python] by Michael Ferguson - to be installed in baxter_ws/src directory
4. [Baxter Simulator] - v1.1 (ONLY needed if NOT working with actual baxter robot)

### 2. Package Contents ###

The `baxter_two` package consists of following main files:

* **baxter_two.launch**: A launch file that starts the following nodes:
	* joint_action_trajectory_server.py (required for MoveIt!)
	* MoveIt! with rviz
	* baxter_img.cpp
	* baxter_pnp.py

* **baxter_img.cpp**: subscribes to the `/cameras/right_hand_camera/image` topic, calculates object locations using [OpenCV] tools, and publishes this information to `baxter_pnp` node via a `PoseArray()` message

* **baxter_pnp.py**: subscribes to the topic (called `Dpos`) published by *baxter_img*, and calls `PlanningSceneInterface` and `MoveGroupInterface` classes from `moveit_python` bindings to add collision objects and move baxter's arms respectively
	* Allows both arms to be moved at the same time
	* Allows velocity scaling using `max_velocity_scaling_factor`
	* Uses `baxter_interface` for gripper control

### 3. Step-by-step guide ###

This section gives a step-by-step approach to run a successful baxter pick and place with this package. 
1. Items needed - a table of height ~ 0.6 m; square blocks (with green top surface); a baxter robot with electric parallel grippers; an ethernet cable (to connect baxter to computer)
2. Set up [networking] with baxter
3. Install the required packages as outlined in **Prerequisites** section
4. Clone this package to your `baxter_ws/src` directory
5. [source] baxter.sh file, enable the robot and launch `baxter_two.launch` as follows
	
```
cd ~/baxter_ws/
. baxter.sh
rosrun baxter_tools enable_robot.py -e
roslaunch baxter_two baxter_two.launch
```

### 4. rqt_graph ###

The following rqt_graph shows a ROS computation graph for baxter pick and place highlighting all important nodes and topics during operation:

<img src="" align="middle" width="300">

## Future Work ##

* **Camera Calibration**: We are looking into improving the camera calibration. Currently, variations in surrounding lighting affect the object detection. Using more finely-tuned methods of calibrating the camera will minimize the lighting issue. 

* **GUI Development**: We are also looking to create a Graphical User Interface (GUI) that allows the user to click an object in the virtual world (e.g in [Gazebo]), and pick and place that particular object. This GUI is currently under development.

[Baxter SDK]: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup
[MoveIt!]: http://moveit.ros.org/install/
[moveit_python]: https://github.com/mikeferguson/moveit_python
[Baxter Simulator]: http://sdk.rethinkrobotics.com/wiki/Simulator_Installation
[OpenCV]: http://opencv.org/
[networking]: http://sdk.rethinkrobotics.com/wiki/Networking
[source]: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup

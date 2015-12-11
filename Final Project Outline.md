Group 2 - Final Project
=========================



##1. Summary##

This *baxter_two* package is used to control Baxter, a robot created by [Rethink Robotics], allowing Baxter to recognize several objects placed on a table, pick up the objects, and move them to a specified goal location. 


##2. Overview##

This metapackage utilizes tools offered in several different existing packages. The primary package used in Baxter's pick and place movements is the [MoveIt!] motion planning framework, paired with the [moveit_python] Python bindings created by Michael Ferguson. Additionally, [OpenCV] was used to locate the objects using one of Baxter's built-in cameras. This pick-and-place technique utilizes the built-in camera on Baxter's right arm to search for green cubes in the task space. Large green cubes are stacked on top of one another in the goal space, while small green cubes are simply placed in the goal space. Positions of objects can be dynamically repositioned and reoriented, and Baxter will continue to pick and place these object until there are no more object in the task space. 

There are two nodes that are run in this package. The first node, *baxter_img*, subscribes to the `/cameras/right_hand_camera/image` topic, calculates cube locations using OpenCV tools, and publishes this information to the second node, *baxter_pnp*. *baxter_pnp* then subscribes to the topic published to by *baxter_img*. Then, it utilizes `PlanningSceneInterface` and `MoveGroupInterface` from *moveit_python* to create Baxter's world including the detected objects and plan/execute paths. 



##3. Tutorial##


####A. Initializing your workspace####

Before using this package, Baxter himself, as well as your work station, must be configured. follow the instructions on the Rethink Robotics [Baxter setup instructions].

Next, the Baxter Simulator must be installed. Instructions for this are also found on the Rethink Robotics website [here]. Before following these instructions, unpack the included tar.gz file into your workspace. When following the instructions, be sure to install the correct version (version 1.2 of the simulator is released, but the SDK currently has not been updated, and compatible firmware for Baxter has not been released). In the instructions, before running `wstool update`, open the *baxter_simulator.rosinstall* file, and delete the first four lines. This will allow `wstool update` to function without having access to the simulator repo.

Once the simulator is installed, *MoveIt!* must be cloned to the working directory following [this tutorial]. Be sure to install the correct package for your ROS distribution. Next, the [moveit_python] bindings must be cloned through the same method. 

Finally, *OpenCV* must be installed. Follow [these instructions] for setting up OpenCV.

Once this process is complete, clone this repository to the designated workspace, and re-build using *catkin_make*. 

####B. Running the Demo####

First, initiatilize the communication between the Baxter simulator and your computer. Connect Baxter's ethernet cable to the computer containing the Baxter workspace. Then, assure the computer is connected to the network established when configuring Baxter. Once the connections are established, run the following commands:
```bash
cd ~/baxter_ws/
. baxter.sh
rosrun baxter_tools enable_robot.py -e
roslaunch baxter_two baxter_two.launch
```
**NOTE:** to run this demo on the Baxter simulation, use `roslaunch baxter_gazebo baxter_world.launch` to start up the simulator, and replace `. baxter.sh` with `. baxter.sh sim`.



##4. Future Work##

One potential area of improvement for this package is in the camera calibration. currently, variations in surrounding lighting affect the object detection success. Using more finely-tuned methods of calibrating the camera will minimize the lighting issue. 

Another area is in creating a Graphical User Interface that allows the user to click an object in the virtual world created by Baxter, and pick and place that particular item. In the current state, Baxter will pick up the object that it detects to be closest to itself. Currently, a GUI is under development that utilizes Gazebo pugin functionality. This GUI, when complete, would allow the user to choose an object presented in the Gazebo world, and click a button to execute the pick-and-place action.





[Rethink Robotics]: http://www.rethinkrobotics.com/baxter/
[MoveIt!]: https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial#tutorial
[Baxter setup instructions]: http://sdk.rethinkrobotics.com/wiki/Getting_Started
[here]: http://sdk.rethinkrobotics.com/wiki/Simulator_Installation
[moveit_python]: https://github.com/mikeferguson/moveit_python
[this tutorial]: https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial#tutorial
[OpenCV]: http://opencv.org/
[these instructions]: http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html
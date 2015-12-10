Group 2 - Final Project
=========================



##1. Summary##

This *baxter_two* package is used to control Baxter, a robot created by [Rethink Robotics], allowing Baxter to recognize several objects placed on a table, pick up the objects, and move them to a specified goal location. 


##2. Overview##

This metapackage utilizes tools offered in several different existing packages. The primary package used in Baxter's pick and place movements is the [MoveIt!] motion planning framework, paired with the [moveit_python] Python bindings created by Michael Ferguson. Additionally, [OpenCV] was used to locate the objects using one of Baxter's built-in cameras. 



##3. Tutorial##


###A. Initializing your workspace###

Before using this package, Baxter himself, as well as your work station, must be configured. follow the instructions on the Rethink Rootics [Baxter setup instructions].

Next, the Baxter Simulator must be installed. Instructions for this are also found on the Rethink Robotics website [here]. Before following these instructions, unpack the included tar.gz file into your workspace. When following the instructions, be sure to install the correct version (version 1.2 of the simulator is released, but the SDK currently has not been updated, and compatible firmware for Baxter has not been released). In the instructions, before running `wstool update`, open the *baxter_simulator.rosinstall* file, and delete the first four lines. This will allow `wstool update` to function without having access to the simulator repo.

Once the simulator is installed, *MoveIt!* must be cloned to the working directory following [this tutorial], and the *moveit_python* bindings must be cloned through the same method. Once this process is complete, the workspace must be re-built using *catkin_make*. 

It is possible to execute this pick-and-place action using a simulated environment.

First, in the designated Baxter workspace, initiatilize the communication between the Baxter simulator and your computer by running the following command:
```bash
cd ~/baxter_ws/
. baxter.sh sim
```


##4. Future Work##





[Rethink Robotics]: http://www.rethinkrobotics.com/baxter/
[MoveIt!]: https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial#tutorial
[Baxter setup instructions]: http://sdk.rethinkrobotics.com/wiki/Getting_Started
[here]: http://sdk.rethinkrobotics.com/wiki/Simulator_Installation
[moveit_python]: https://github.com/mikeferguson/moveit_python
[this tutorial]: https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial#tutorial
[OpenCV]: http://opencv.org/
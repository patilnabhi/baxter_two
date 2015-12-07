#Group 2 - Final Project

##1. Summary

This *bax_two* package is used to control Baxter, a robot created by [Rethink Robotics], allowing Baxter to recognize several objects placed on a table, pick up the objects, and move them to a specified goal location. 


##2. Overview

This metapackage utilizes tools offered in several different existing packages. The primary package used in Baxter's pick and place movements is the [MoveIt!] motion planning framework, paired with the [moveit_python] Python bindings created by Michael Ferguson. Additionally, [OpenCV] was used to locate the objects using one of Baxter's built-in cameras. 



##3. Tutorial

Before using this package, *MoveIt!* must be cloned to the working directory following [this tutorial], and the *moveit_python* bindings must be cloned through the same method. Once this process is complete, the workspace must be re-built using *catkin_make*. 

*insert more information here*







[Rethink Robotics]: http://www.rethinkrobotics.com/baxter/
[MoveIt!]: https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial#tutorial
[moveit_python]: https://github.com/mikeferguson/moveit_python
[this tutorial]: https://github.com/RethinkRobotics/sdk-docs/wiki/MoveIt-Tutorial#tutorial
[OpenCV]: http://opencv.org/
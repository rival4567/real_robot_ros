# ROS with Gazebo

## Difficulty: Intermediate

---

This page is just to quick overview of the simulation and visualization tools in ROS. 

> **Note**: ROS and Gazebo together are a great combination to simulate how your algoirthm would work in real time scenarios. 

## Gazebo Simulator

- Robot simulation is an essential tool in every roboticist's toolbox.

- A robust physics engine, high-quality graphics, and convenient programmatic and graphical interfaces, makes Gazebo a top Choice for 3D Simulator.

**.world** File: The file used to describe a collection of objects (such as buildings, tables, and lights), and global parameters including the sky, ambient light, and physics properties.

### Reference

1. [Gazebo Tutorials](http://gazebosim.org/tutorials)

---

## RViz

- Visualizing sensor information is an important part in developing and debugging controllers. 

- Rviz is a powerful 3D visualization tool in ROS that will hep you do exactly that.

- It allows the user to view the simulated robot model, log sensor information from the robot's sensors, and replay the logged sensor information.

### Reference

1. [ROS Wiki: RViz](http://wiki.ros.org/rviz)
1. [Gazebo: Visualization and logging](http://gazebosim.org/tutorials?tut=drcsim_visualization&cat=drcsim)

---

## URDF

- The Unified Robot Description Format (URDF) contains a number of XML specifications for robot models, sensors, scenes, etc. 

- It describes the position of all the joints, sensors, type of joints, structure of the robot base, arm etc. 

### Reference

1. [ROS Wiki: URDF overview](http://wiki.ros.org/urdf)
1. [ROS Wiki: URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

---

## XACRO

- Xacro (XML Macros) Xacro is an XML macro language. 
- With xacro, you can construct shorter and more readable XML files by using macros that expand to larger XML expressions.
- Xacro is useful when the structure of the robot is complex so instead of describing the whole structure in an urdf we can divide the structure in small parts and call those macro files in the main xacro file.
- Xacros also make it easier to define common structures. For example, let's say the robot has 2 wheels, we just need to make macros of a cylindrical structure(wheels), call it in the main xacro file and then define 2 different joints using the same structure but giving different joint location. 

### Reference

1. [ROS Wiki: Using Xacro to Clean Up a URDF File](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File)
1. [ROS Wiki: Xacro overview](http://wiki.ros.org/xacro)

---

## Gazebo Plugins

- A Gazebo plugin needs to be added to your URDF that actually parses the transmission tags and loads the appropriate hardware interfaces and controller manager. 
- Plugins basically replicate exact architecture of the sensors in use or the control system used to control the movement of the robot. 

### What are Transmission Tags?

- Transmission tags are used to link actuators to joints. 
- If the transmission tags the joints won't move in Gazebo and they will be considered as stationary objects.
- We need to define transmission for every dynamic(moving) joint. 

### Reference

1. [Gazebo tutorials: ROS Control](http://gazebosim.org/tutorials/?tut=ros_control#Aboutros_control)

---


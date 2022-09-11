# Configuring your ROS environment

**Goal**: This tutorial will show you how to prepare your ROS environment.

## Prerequisites

Before starting these tutorials please complete installation as described in the [ROS installation instructions](http://wiki.ros.org/ROS/Installation).

## Managing Your Environment

During the installation of ROS, you will see that you are prompted to source one of several setup.*sh files, or even add this 'sourcing' to your shell startup script. This is required because ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier.

If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set: 

```bash
$ printenv | grep ROS
```

```bash
ROS_VERSION=1
ROS_PYTHON_VERSION=3
ROS_PACKAGE_PATH=/home/ubuntu/workspace/src:/opt/ros/noetic/share
ROSLISP_PACKAGE_DIRECTORIES=/home/ubuntu/workspace/devel/share/common-lisp
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ROS_MASTER_URI=http://localhost:11311
ROS_ROOT=/opt/ros/noetic/share/ros
ROS_DISTRO=noetic
```

If they are not then you might need to 'source' some setup.*sh files. 

```bash
$ source /opt/ros/noetic/setup.bash
```

> **NOTE**: You will need to run this command on every new shell you open to have access to the ROS commands, unless you add this line to your .bashrc.

To add this line to .bashrc run this command:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

To undo this (to change to another distro) in Linux and macOS, locate your system’s shell startup script and remove the appended source command.

## Summary

The ROS development environment needs to be correctly configured before use. This can be done in two ways: either sourcing the setup files in every new shell you open, or adding the source command to your startup script.

If you ever face any problems locating or using packages with ROS, the first thing you should do is check your environment variables and ensure they are set to the version and distro you intended.

---
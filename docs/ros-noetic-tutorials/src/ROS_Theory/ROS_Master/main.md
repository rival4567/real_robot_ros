# ROS Master

- The `ROS Master` provides naming and registration services to the rest of the nodes in the ROS system.

- As you know ROS Nodes are building blocks of any ROS Application. A single ROS Application may have multiple ROS Nodes which communicate with each other. 

- The role of the ROS Master is to enable individual ROS nodes to locate one another.

- Once these nodes have located each other they communicate with each other peer-to-peer.  

- You can say, communication is established between nodes by the ROS Master. So, without `ROS Master` running ROS Nodes can not communicate with each other.

## Start ROS Master

To start `ROS Master` you just have to enter the following command in the terminal.

```bash
roscore
```

![ros-master.png](ros-master.png)

`roscore` is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate.


So [roscore](http://wiki.ros.org/roscore) will start the following:

1. ROS Master
1. ROS Parameter Server
1. `rosout` Logging Node

In the preceding output, you can see information about the computer, parameter which list the name (noetic) and version number of ROS distribution, and some other information.  

## Reading Assignment

1. [ROS Wiki - Master](http://wiki.ros.org/Master)
1. [ROS Wiki - roscore](http://wiki.ros.org/roscore)
---
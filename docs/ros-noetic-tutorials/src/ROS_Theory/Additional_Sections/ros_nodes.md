# Understanding ROS nodes

**Goal**: Learn about the function of nodes in ROS, and the tools to interact with them.

## Background

### 1 The ROS graph

Over the next few tutorials, you will learn about a series of core ROS concepts that make up what is referred to as the “ROS graph”.

The ROS graph is a network of ROS elements processing data together at one time. It encompasses all executables and the connections between them if you were to map them all out and visualize them.

### 2 Nodes in ROS

Each node in ROS should be responsible for a single, module purpose (e.g. one node for controlling wheel motors, one node for controlling a laser range-finder, etc). Each node can send and receive data to other nodes via topics, services, actions, or parameters.

![node-graph.gif](node-graph.gif)

A full robotic system is comprised of many nodes working in concert.

## Prerequisites

As always, don’t forget to source ROS in every new terminal you open. Also, run `roscore` in a
separate terminal.

## Tasks

### 1 rosrun

The command `rosrun` launches an executable from a package.

```bash
rosrun <package_name> <executable_name>
```

To run turtlesim, open a new terminal, and enter the following command:

```bash
rosrun turtlesim turtlesim_node
```

Here, the package name is `turtlesim` and the executable name is `turtlesim_node`.

We still don’t know the node name, however. You can find node names by using `rosnode list`

### 2 rosnode list

`rosnode list` will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.

Open a new terminal while turtlesim is still running in the other one, and enter the following command:

```bash
rosnode list
```

The terminal will return the node name:

```bash
/rosout
/turtlesim
```

Open another new terminal and start the teleop node with the command:

```bash
rosrun turtlesim turtle_teleop_key
```

Here, we are searching the `turtlesim` package again, this time for the executable named `turtle_teleop_key`.

Return to the terminal where you ran `rosnode list` and run it again. You will now see the names of two active nodes:

```bash
/rosout
/teleop_turtle
/turtlesim
```

#### 2.1 Remapping

[Remapping](http://wiki.ros.org/Remapping%20Arguments) allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values. In the last tutorial, you used remapping on `turtle_teleop_key` to change the default turtle being controlled.

Now, lets reassign the name of our `/turtlesim` node. In a new terminal, run the following command:

```bash
rosrun turtlesim turtlesim_node __name:=my_turtle
```

Since you’re calling `rosrun` on turtlesim again, another turtlesim window will open. However, now if you return to the terminal where you ran `rosnode list`, and run it again, you will see three node names:

```bash
/my_turtle
/rosout
/teleop_turtle
/turtlesim
```

### 3 rosnode info

Now that you know the names of your nodes, you can access more information about them with:

```bash
rosnode info <node_name>
```

To examine your latest node, `my_turtle`, run the following command:

```bash
rosnode info /my_turtle
```

`rosnode info` returns a list of subscribers, publishers, services, and actions (the ROS graph connections) that interact with that node. The output should look like this:

```bash
Node [/my_turtle]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /turtle1/color_sensor [turtlesim/Color]
 * /turtle1/pose [turtlesim/Pose]

Subscriptions: 
 * /turtle1/cmd_vel [geometry_msgs/Twist]

Services: 
 * /clear
 * /kill
 * /my_turtle/get_loggers
 * /my_turtle/set_logger_level
 * /reset
 * /spawn
 * /turtle1/set_pen
 * /turtle1/teleport_absolute
 * /turtle1/teleport_relative


contacting node http://ros-noetic:44953/ ...
Pid: 878
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (40097 - 10.104.247.68:60584) [18]
    * transport: TCPROS
 * topic: /turtle1/cmd_vel
    * to: /teleop_turtle (http://ros-noetic:44321/)
    * direction: inbound (58058 - ros-noetic:47853) [17]
    * transport: TCPROS
```

Now try running the same command on the `/teleop_turtle node`, and see how its connections differ from `my_turtle`.

You will learn more about ROS graph connection concepts in the upcoming tutorials.

## Summary

A node is a fundamental ROS element that serves a single, modular purpose in a robotics system.

In this tutorial, you utilized nodes created from the `turtlesim` package by running the executables `turtlesim_node` and `turtle_teleop_key`.

You learned how to use `rosnode list` to discover active node names and `rosnode info` to introspect on a single node. These tools are vital to understanding the flow of data in a complex, real-world robot system.

---
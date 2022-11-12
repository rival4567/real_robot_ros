# Problem Statement

- The objective of this task is to spawn two turtles in a **turtlesim** window and make one turtle follow another.

> **Note**: The name of turtles should be `turtle1` and `turtle2` respectively with `turtle2` following `turtle1`. `turtle1` should be spawned at default coordinates (5.544445, 5.544445, 0) and `turtle2` at (1.0, 1.0, 0.0).

- You can do this by creating a node name, `node_catch_the_turtle` with a python script, `node_catch_the_turtle.py`.

**Optional**:

- Change the color of the pen of turtles.
    - **turtle1**
        - `r: 255`
        - `g: 255`
        - `b: 0`
        - `width: 5`
    - **turtle2**
        - `r: 255`
        - `g: 0`
        - `b: 0`
        - `width: 4`

- Change the background to **forestgreen**. 
    - `background_r: 34`
    - `background_g: 139`
    - `background_b: 34`

## Procedure

1. First, create a package name `pkg_task0`, within your catkin workspace. Once done, compile and source the packages.

```bash
cd ~/workspace
catkin_make
source devel/setup.bash
```

1. Within this package, you should have a `scripts` folder inside which you'll create a python script, named `node_catch_the_turtle.py`.

> **Note**: Fill the script with proper programming ethics. Doing this will help us understand your code better and quicker than usual.

1. After completing the python script. Make it executable, if it isn't already. To do that, enter the following code.

```bash
chmod +x ~/workspace/src/pkg_task0/scripts/node_catch_the_turtle.py
```

1. Before executing make sure that `roscore` is running along with `turtlesim_node`. You can either run them in separate terminals or simply create a `task0.launch` file inside the `~/workspace/src/pkg_task0/launch/` folder. Launch file can run multiple nodes unlike a python/cpp script. Run the launch file, enter,

```bash
roslaunch pkg_task0 task0.launch 
```

- This should run these processes in parallel.
    - roscore
    - turtlesim_node
    - turtle_teleop_key
    - node_catch_the_turtle.py

## Hints
- You can use linear velocity as well as angular velocity with some combination to get this done. 
- Keep tracking the distance travelled so as to know when to stop.

---

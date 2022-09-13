# Problem Statement

- The objective of this task is to spawn a turtle in a **turtlesim** window and move it in a circle using ros nodes.
- You can do this by creating a node name, `node_do_circle` with a python script, `node_do_circle.py`.

- Change the color of the pen of the turtle.
    - **turtle1**
        - `r: 255`
        - `g: 255`
        - `b: 0`
        - `width: 5`

    > You will need to use `rosservices` to do this part of the task.

- Change the background to **forestgreen**. 
    - `background_r: 34`
    - `background_g: 139`
    - `background_b: 34`

    > You will need to use `rosparam` to do this part of the task.

## Procedure

1. First, create a package name `pkg_move_turtle`, within your catkin workspace. Once done, compile and source the packages.

```bash
cd ~/workspace
catkin_make
source devel/setup.bash
```

1. Within this package, you should have a `scripts` folder inside which you'll create a python script, named `node_do_circle.py`.

> **Note**: Fill the script with proper programming ethics. Doing this will help us understand your code better and quicker than usual.

1. After completing the python script. Make it executable, if it isn't already. To do that, enter the following code.

```bash
chmod +x ~/workspace/src/pkg_move_turtle/scripts/node_do_circle.py
```

1. Before executing make sure that `roscore` is running along with `turtlesim_node`. You can either run them in separate terminals or simply create a `move_in_a_circle.launch` file inside the `~/workspace/src/pkg_move_turtle/launch/` folder. Launch file can run multiple nodes unlike a python/cpp script. Run the launch file, enter,

```bash
roslaunch pkg_move_turtle move_in_a_circle.launch 
```

- This should run these processes in parallel.
    - roscore
    - turtlesim_node
    - node_do_circle.py

## Hints
- You can use linear velocity as well as angular velocity with some combination to get this done. 
- Keep tracking the distance travelled so as to know when to stop.

---
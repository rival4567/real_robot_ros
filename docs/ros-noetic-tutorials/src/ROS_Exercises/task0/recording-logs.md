# Recording Logs

- ROS allows us to record a log of the messages that occurred in a given time period. This is like recording a data stream. The ROS utility which does this is called rosbag, and the command to capture the data is `rosbag record`.

- Create a folder called bag_files in your package as a save destination for the bag files.

- You can run the rosbag record command separately on the command line. But to not loose any data you will have to start recording precisely at the same moment your turtle starts moving. Hence it is a much more preferable option to include the rosbag recording in your launch file itself.

- Add the following lines to your launch file to have the rosbag record run in parallel with your task. Some parameters are explained below:

```xml
    <arg name="record" default="false"/>
    <arg name="duration" default="100"/>
    <arg name="rec_name" default="turtle_record.bag"/>

    <group if="$(arg record)">
        <node name="rosbag_record_turtle" pkg="rosbag" type="record"
            args="record -O $(find pkg_task0)/bag_files/$(arg rec_name) --duration=$(arg duration) --chunksize=10 /turtle1/cmd_vel /turtle1/pose /turtle2/cmd_vel /turtle2/pose" output="screen" />
    </group>
```

> **Note**: Make sure you have added these line before `</launch>` line. And the 3 nodes, the turtlesim, teleop_key and your python script node, are already present within this launch file, for desire recording.

- The `arg_name` tags are roslaunch parameters, meaning they can be called while calling your roslaunch file, for example:

```bash
roslaunch pkg_task0 task0.launch record:=true rec_name:=my_turtle.bag
```

- This command will.. 
    - Start recording.
    - Should start turtlesim and turtle_teleop_key node.
    - Should start your python script node.
    - And name the resultant bag file as __my_turtle.bag__.

while still retaining the default value (in seconds) for the duration parameter.

Thus using these parameters along with your launch file will record a bag file of appropriate `duration`. Also, Make sure that the task is completed within the recording duration. **If not then shorten the duration during which you run turtle1 using teleop_key.**

- If the rosbag has started, the message:
```bash
process[rosbag_record_turtle-5]: started with pid [1056]
```
will appear on your terminal within the roslaunch output window.

> **Note**:  bag files with the same name will be overwritten by the rosbag utility without a prompt/warning. Make sure you provide proper name for each iteration if you want to save them all. 

---
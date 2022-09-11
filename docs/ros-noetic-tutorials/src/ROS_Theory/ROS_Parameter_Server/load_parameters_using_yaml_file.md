# Load Parameters using YAML file

In this section we will learn how to load your own parameters in ROS Parameter Server using a YAML File.

## Steps

1. Navigate to `pkg_ros_basics`.
    ```bash
    cd ~/workspace/src/pkg_ros_basics
    ```
    OR
    ```bash
    roscd pkg_ros_basics
    ```

    > **NOTE**: `roscd` will work only if you have sourced `setup.bash` of your catkin workspace.

1. Create a `config` folder for your Python scripts and navigate into the folder.
   ```bash
   mkdir config
   cd config
   ```

1. Create a configuration YAML file called `config_my.yaml`.
   ```bash
   touch config_my.yaml
   ```

1. Open the script in any text-editor and start editing.
   ```bash
   gedit config_my.yaml
   ```

1. Now fill your config file.
   ```yaml
   # Comment: config_my.yaml Configuration
   details:
       name:
           first: "Heisenberg" # First Name
           last: "White" # Last Name
        contact:
            address: "ABQ Street, ABQ" # Address
            phone: 77777    # Contact
   ```
   <center><a href="config_my.yaml" download><button>Download</button></a></center>
   
   - ROS Build system will create a Python Dictionary called `details`.

   - This dictionary will have two keys,

        1. Dictionary `name`
        2. Dictionary `contact`

    - In your ROS Node you can use `rospy` to get parameters stored in this `config_my` dictionary.

      ```python
      param_config_my = rospy.get_param('details')

      first_name = param_config_my['name']['first']
      phone = param_config_my['contact']['phone']
      ```

   <br />

1. Now if you want to load the parameters defined in the YAML file you have to first start the ROS Parameter Server.

    Open up a new terminal and enter the following.
    ```bash
    roscore
    ```

1. Now load your parameters.
    ```bash
    rosparam load config_my.yaml
    ```

1. Now get the list of parameters loaded in your ROS Parameter Server.
   ```bash
   rosparam list
   ```
   Output:
   ```bash
   /details/contact/address
   /details/contact/phone
   /details/name/first
   /details/name/last
   /rosdistro
   /roslaunch/uris/host_ros_noetic__35261
   /rosversion
   /run_id
   ```
   Here you can see the first four parameters are loaded from our `config_my.yaml` file.

1. Now to view the content of any parameter do the following.
   ```bash
   rosparam get /details/contact/phone
   ```
   Output:
   ```bash
   77777
   ```
   This is the value which we defined in the `config_my.yaml` file.

   ![rosparam-load-config_my.png](rosparam-load-config_my.png)

---
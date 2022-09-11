# Example #1: ROS Node to Get and Set Parameters

## Aim
To write a ROS Node to read `config_my.yaml` file loaded in ROS Parameter Server ([done here](load_parameters_using_yaml_file.html#Load-Parameters-using-YAML-file)), print it on the console and modify the phone number.

## Code

`node_param_get_set.py`
```python
#!/usr/bin/env python3

import rospy


def main():    
    
    # 1. Make the script a ROS Node.
    rospy.init_node('node_param_get_set', anonymous=True)

    # 2. Read from Parameter Server
    rospy.loginfo("Reading from Parameter Server.")

    
    param_config_my = rospy.get_param('details')    # Get all the parameters inside 'details'

    # Store the parameters in variables
    first_name = param_config_my['name']['first']
    last_name = param_config_my['name']['last']
    address = param_config_my['contact']['address']
    phone = param_config_my['contact']['phone']

    # Print the parameters
    rospy.loginfo(">> First Name: {}".format(first_name))
    rospy.loginfo(">> Last Name: {}".format(last_name))
    rospy.loginfo(">> Address: {}".format(address))
    rospy.loginfo(">> Phone: {}".format(phone))

    # 3. Modify the Phone Number 

    rospy.set_param('/details/contact/phone', 55555)        # Modify only Phone Number in Parameter Server
    new_phone = rospy.get_param('/details/contact/phone')   # Get only Phone Number from Parameter Server
    rospy.loginfo(">> New Phone: {}".format(new_phone))     # Print the new Phone Number
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

<center><a href="node_param_get_set.py" download><button>Download</button></a></center>

<br />

> **NOTE**: Make sure you make the `pkg_ros_basics node_param_get_set.py` script executable.

Output:

```bash
rosrun pkg_ros_basics node_param_get_set.py
```

![node-param-get-set-output.png](node-param-get-set-output.png)

- The code is self-explanatory.
- If you are not able to understand the code feel free to seek help from us.
---
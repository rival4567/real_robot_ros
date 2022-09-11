# Create a ROS Package

- This tutorial will demonstrate how to use the `catkin_create_pkg` script to create a new catkin package, and what you can do with it after it has been created.


1. First, navigate to the source space directory of the catkin workspace you've created.
   ```bash
   cd ~/workspace/src
   ```
2. Now, use the catkin_create_pkg script to create a new package called pkg_ros_basics which depends on std_msgs, roscpp, and rospy:
   ```bash
   catkin_create_pkg pkg_ros_basics std_msgs rospy roscpp
   ```
      - This will create a beginner_tutorials folder which contains a `package.xml` and a `CMakeLists.txt`, which have been partially filled out with the information you gave `catkin_create_pkg`.

      - `catkin_create_pkg` requires that you give it a `package_name` and optionally a list of dependencies on which that package depends:
        ```bash
        catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
        ```

3. Now, you need to build the packages in the catkin workspace:
   ```bash
   cd ~/workspace
   catkin_make
   ```

- Inside the package, there are `src` folder, `package.xml`, `CMakeLists.txt`, and the `include` folders.
   - **CMakeLists.txt**: This file has all the commands to build the ROS source code inside the package and create the executable. For more information about CMakeLists visit [here](http://wiki.ros.org/catkin/CMakeLists.txt).
   - **package.xml**: This is an XML file. It mainly contains the package dependencies, information, and so forth.
   - **src**: The source code of ROS packages are kept in this folder.

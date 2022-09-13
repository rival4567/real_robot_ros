# Create a Catkin Workspace
1. Open up the terminal `shortcut key: ctrl+alt+t`.
2. Create the root workspace directory. You can name your directory anything we are using `workspace` as the name this time.
```bash
cd ~/
mkdir -p ~/workspace/src
cd workspace
```
3. Run the following command:
```bash
catkin_make
```
4. The `catkin_make` command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a `CMakeLists.txt` link in your `src` folder.
5. Additionally, if you look in your current directory you should now have a 'build' and 'devel' folder.
```bash
ls
```
6. Now to make your workspace visible to ROS. Source the setup file in the devel directory.
```bash
source ~/workspace/devel/setup.bash
```
By doing this, all the packages that you create inside the `src` folder will be visible to ROS.
s
7. This `setup.bash` file of your workspace must be source everytime when you want to use ROS packages created inside this workspace.

8. To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.

```bash
echo $ROS_PACKAGE_PATH
/home/youruser/workspace/src:/opt/ros/noetic/share
```
<hr>

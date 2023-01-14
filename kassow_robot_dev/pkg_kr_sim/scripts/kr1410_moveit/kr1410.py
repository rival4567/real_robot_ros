'''
    kr1410
    =====================
    This file includes MoveIt! methods to control motion
    of KR1410 robot arm and gripper. It includes a class which has 
    variables and methods to move KR1410. User can move KR1410 using 
    joint angles, cartesian path, go_to_pose. This will be controlled
    through services, actions and topics.
    *******************************************************************
    Author: Shivam Shivam (shivam.shivam@stud.th-deg.de)
    *******************************************************************
    University: Technologie Hochschule Deggendorf
    ---------------------------------------------
'''

from sys import argv
import copy
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import actionlib
import rospkg
import yaml

from std_srvs.srv import Empty


class KR1410Moveit():
    '''This class moves the kr1410 arm using MoveIt!
        ``set_joint_angles()``, ``go_to_pose()``,
        ``ee_cartesian_translation()`` are used to move kr1410 arm.
        ``save_best_planned_trajectory``, ``play_planned_path_from_file``
        to save and play path from a yaml file. Some other methods
        are also present to add, attach, detach, remove objects
        from the planning scene.
    :param arg_robot_name: Controls robotic arm with name
    :type arg_robot_name: str
    '''
    # pylint: disable=too-many-instance-attributes

    def __init__(self, arg_robot_name, controller):
        '''Constructor
        '''

        self._robot_ns = '/' + arg_robot_name if arg_robot_name else ''

        self._planning_group = controller

        # Initialize `moveit_commander`_ node
        moveit_commander.roscpp_initialize(argv)

        # Kinematic model and the robot's current joint states
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns)

        # Instantiate a `PlanningSceneInterface`_ object. This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:

        self._scene = moveit_commander.PlanningSceneInterface(
            ns=self._robot_ns)

        # Instantiate a `MoveGroupCommander`_ object. This object is an interface
        # to a planning group (group of joints).
        # This interface can be used to plan and execute motions:
        self.group = moveit_commander.MoveGroupCommander(
            self._planning_group, robot_description=self._robot_ns +
            "/robot_description", ns=self._robot_ns)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in RViz:
        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1)

        # Creating a simple action client to send goals.
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory',
            moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        # Other class variables
        self._planning_frame = self.group.get_planning_frame()
        self._eef_link = self.group.get_end_effector_link()
        self.group_names = self._robot.get_group_names()
        self._box_name = ''

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_group)
            + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self.group_names)
            + '\033[0m')

        # To save planned path into a directory
        self._pkg_path = rospkg.RosPack().get_path('pkg_kr_sim')
        self.file_path = self._pkg_path + \
            '/config/saved_trajectories/' + self._planning_group
        rospy.loginfo("Package Path: {}".format(self.file_path))

        rospy.loginfo('\033[94m' + " >>> KR1410Moveit init done." + '\033[0m')

    def clear_octomap(self):
        '''Calling internal MoveIt service for controlling multiple kr140
        :return: Create a handle to ROS service for invoking calls
        :rtype: class:`ServiceProxy`
        '''
        clear_octomap_service_proxy = rospy.ServiceProxy(
            self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    @staticmethod
    def list_degrees_to_radians(list_degrees):
        '''Converts list of degrees to radians
        :param list_degrees: List of degrees
        :type list_degrees: list
        :return: List of radians
        :rtype: list
        '''
        list_in_radians = []
        for list_d in list_degrees:
            list_in_radians.append(math.radians(list_d))
        return list_in_radians

    def set_joint_angles(self, arg_list_joint_angles):
        '''Setting joint angles for `kr1410_planning_group` all links
        :param arg_list_joint_angles: list of joint angles to set
        :type arg_list_joint_angles: list
        :return: `True` if successful
        :rtype: bool
        '''
        list_joint_values = self.group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        # Move to desired joint angles
        self.group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self.group.plan()
        flag_plan = self.group.g\o(wait=True)

        list_joint_values = self.group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self.group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        # Calling `stop()` ensures that there is no residual movement.
        self.group.stop()
        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts=10):
        '''Sets joint angles in multiple attempts
        :param arg_list_joint_angles: list of joint angles to set
        :type arg_list_joint_angles: list
        :param arg_max_attempts: Number of attempts, defaults to 10
        :type arg_max_attempts: int, optional
        '''
        number_attempts = 0
        flag_success = False
        self.group.set_planner_id("RRTConnect")
        self.group.set_planning_time(5)

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()
            self.group.clear_pose_targets()
            if number_attempts > 1:
                self.wait_for_robot_state_update(
                    self.group.get_current_joint_values())

    def go_to_pose(self, arg_pose):
        '''Go to desired pose for the end-effector
        :param arg_pose: position in x, y, z and orientation x, y, z, w
        :type arg_pose: geometry_msgs.msg.Pose
        :return: `True` if successful
        :rtype: bool
        '''
        pose_values = self.group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        # Move to target position
        self.group.set_pose_target(arg_pose)
        flag_plan = self.group.go(wait=True)  # wait=False for Async Move

        pose_values = self.group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self.group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if flag_plan:
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found."
                + '\033[0m')

        # Calling 'stop()' ensures that there is no residual movement.
        self.group.stop()
        # Clearing targets after planning of poses.
        self.group.clear_pose_targets()
        return flag_plan

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z, synchronous="True"):
        '''This method use cartesian path to translate in x, y, z direction
        :param trans_x: translation in x
        :type trans_x: int
        :param trans_y: translation in y
        :type trans_y: int
        :param trans_z: translation in z
        :type trans_z: int
        :param synchronous: Move kr1410 arm asynchronously if "False", defaults to "True"
        :type synchronous: str, optional
        '''
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self.group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, _fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold

        rospy.loginfo("Path computed successfully. Moving the arm.")

        # Deleting first 2 waypoints to remove time error.
        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self.group.execute(plan, wait=synchronous)

    def save_planned_trajectories(self, arg_list_joint_angles, file_name="temp.yaml",
                                  arg_max_attempts=5):
        '''This method is used to save planned trajectories in a yaml file to save time
        :param arg_list_joint_angles: list of joint angles
        :type arg_list_joint_angles: list
        :param file_name: name of file to save as, defaults to "temp.yaml"
        :type file_name: str, optional
        :param arg_max_attempts: Maximum attemps to try, defaults to 5
        :type arg_max_attempts: int, optional
        '''
        number_attempts = 0
        flag_plan = False
        while number_attempts < arg_max_attempts and not flag_plan:
            number_attempts += 1
            self.group.set_joint_value_target(arg_list_joint_angles)
            self._computed_plan = self.group.plan()
            flag_plan = self.group.go(wait=True)
            self.group.stop()

        file_path = self.file_path + self._robot_ns + '_' + file_name

        with open(file_path, 'w') as file_save:
            yaml.dump(self._computed_plan, file_save, default_flow_style=True)

        rospy.loginfo("File saved at: {}".format(file_path))

    def save_best_planned_trajectory(self, ref_joint_angles, target_joint_angles,
                                     file_name="temp.yaml", arg_max_attempts=5):
        '''This method saves best planned trajectory over a number of attempts
        :param ref_joint_angles: First set reference joint angles where
            to start planning
        :type ref_joint_angles: list
        :param target_joint_angles: Plan to target joint angles
        :type target_joint_angles: list
        :param file_name: name of file to save as, defaults to "temp.yaml"
        :type file_name: str, optional
        :param arg_max_attempts: Maximum attempt to try, defaults to 5
        :type arg_max_attempts: int, optional
        :return: All computed plans in list
        :rtype: list
        '''
        # pylint: disable-msg=too-many-locals
        number_attempts = 0
        computed_plan = []
        execution_time = []
        # First reach the reference position from where to save trajectory
        self.group.set_joint_value_target(ref_joint_angles)
        self.group.plan()
        self.group.go(wait=True)
        self.group.stop()
        # self.attach_box()
        self.group.set_planning_time(5)
        self.group.set_num_planning_attempts(10000)
        while number_attempts < arg_max_attempts:
            rospy.loginfo("Iteration Number: " + str(number_attempts))
            number_attempts += 1
            # Set target position using joint angles
            self.group.set_joint_value_target(target_joint_angles)
            computed_plan.append(self.group.plan())

        for plan in computed_plan:
            if plan.joint_trajectory.points == []:
                # if plan is empty remove it
                computed_plan.remove(plan)
                continue
            secs = plan.joint_trajectory.points[-1].time_from_start.secs
            nsecs = plan.joint_trajectory.points[-1].time_from_start.nsecs
            # Converting nsecs to secs and adding to secs
            time = secs + nsecs/1e9
            rospy.loginfo(time)
            execution_time.append(time)

        # This loop removes plans which are too fast. KR1410 moves beyond its limits...
        for time, plan in zip(execution_time, computed_plan):
            # if 3.75 < time < 3.79:
            #     execution_time.remove(time)
            #     computed_plan.remove(plan)
            if time < 0.05:
                execution_time.remove(time)
                computed_plan.remove(plan)

        if execution_time:
            rospy.loginfo(execution_time)
            best_time_index = execution_time.index(min(execution_time))
            best_plan = computed_plan[best_time_index]
            rospy.loginfo("Minimum execution time after " + str(arg_max_attempts)
                          + " attempts: " + str(min(execution_time)))

            file_path = self.file_path + self._robot_ns + '_' + file_name
            # Saving File in path
            with open(file_path, 'w') as file_save:
                yaml.dump(best_plan, file_save, default_flow_style=True)

            rospy.loginfo("File saved at: {}".format(file_path))
            return computed_plan

    def play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''This method runs planned trajectory from yaml file
        :param arg_file_path: path of file
        :type arg_file_path: str
        :param arg_file_name: name of file
        :type arg_file_name: str
        :return: `True` if successful, otherwise `False`
        :rtype: bool
        '''
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self.group.execute(loaded_plan)
        return ret

    def hard_play_planned_path_from_file(self, arg_file_path,
                                         arg_file_name, arg_max_attempts=10):
        '''This method runs planned trajectory from yaml file in a
            number of attempts
        :param arg_file_path: path to file
        :type arg_file_path: str
        :param arg_file_name: name of file
        :type arg_file_name: str
        :param arg_max_attempts: Maximum number of attempts to try,
            defaults to 10
        :type arg_max_attempts: int, optional
        '''
        number_attempts = 0
        flag_success = False
        while ((number_attempts <= arg_max_attempts) and
               (flag_success is False)):
            number_attempts += 1
            flag_success = self.play_planned_path_from_file(
                arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            self.group.clear_pose_targets()
            # self.clear_octomap()
            if number_attempts > 1:
                self.wait_for_robot_state_update(
                    self.group.get_current_joint_values())

    def wait_for_robot_state_update(self, prev_robot_state, timeout=4):
        '''Ensuring that robot states are updating if gazebo fails to
            register new joint angles values
        :param prev_robot_state: Current position and orientation of robot
        :type prev_robot_state: list
        :param timeout: time for which to wait (in seconds) before
            returning `False`, defaults to 4
        :type timeout: int, optional
        :return: `True` if state updated, otherwise `False`
        :rtype: bool
        '''
        start = rospy.get_time()
        seconds = rospy.get_time()
        rospy.loginfo("Waiting for robot state to update!")
        # rospy.loginfo("Current Robot State: ")
        # rospy.loginfo(self._curr_state)
        while (seconds - start < timeout) and not rospy.is_shutdown():
            curr_state = self.group.get_current_joint_values()
            # Test if current joint states are updated
            if not curr_state == prev_robot_state:
                return True
            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If exited the while loop without returning then timed out
        return False

    def wait_for_state_update(self, box_is_known=False,
                              box_is_attached=False, timeout=4):
        '''Ensuring Collision Updates Are Received. If the Python node
            dies before publishing a collision object update message,
            the message could get lost and the box will not appear. To
            ensure that the updates are made, waiting until to see the
            changes reflected in the ``get_attached_objects()`` and
            ``get_known_object_names()`` lists. Calling this function
            after adding, removing, attaching or detaching an object
            in the planning scene. Then waiting until the updates have
            been made or ``timeout`` seconds have passed.
        :param box_is_known: checks if box is in RViz env, defaults to False
        :type box_is_known: bool, optional
        :param box_is_attached: checks if box is attached to end
            effector, defaults to False
        :type box_is_attached: bool, optional
        :param timeout: time for which to wait (in seconds) before
            returning False, defaults to 4
        :type timeout: int, optional
        :return: `True` if state updated, otherwise `False`
        :rtype: bool
        '''
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self._scene.get_attached_objects(
                [self._box_name])
            is_attached = True if attached_objects.keys() else False

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self._box_name in self._scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If exited the while loop without returning then timed out
        return False

    def add_box(self, box_name, box_pose, size, timeout=4):
        '''Adding Objects to the Planning Scene. Create a box in the
        planning scene
        :param box_name: name of box
        :type box_name: str
        :param box_pose: pose of box
        :type box_pose: geometry_msgs.msg.PoseStamped
        :param size: size with three dimensions
        :type size: tuple
        :param timeout: timeout in seconds, defaults to 4
        :type timeout: int, optional
        :return: `True` if successful, otherwise `False`
        :rtype: bool
        '''
        self._box_name = box_name
        box_pose.header.frame_id = self._planning_frame  # Planning frame is `world`
        self._scene.add_box(self._box_name, box_pose, size=size)

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, attaching_link='tool_frame', timeout=4):
        '''Attaching objects to the Robot. Attaching link is vacuum_gripper_link.
        :param attaching_link: On which the object will be attached
        :type attaching_link: str, optional
        :param timeout: timeout in seconds, defaults to 4
        :type timeout: int, optional
        :return: Method to wait until the changes are reflected
        :rtype: KR1410Moveit.wait_for_state_update
        '''
        self._scene.attach_box(
            self._eef_link, self._box_name, touch_links=attaching_link)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True,
                                          box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        '''Detaching Objects from the Robot. We can also detach and remove the object
        from the planning scene
        :param timeout: timeout in seconds, defaults to 4
        :type timeout: int, optional
        :return: Method to wait until the changes are reflected
        :rtype: KR1410Moveit.wait_for_state_update
        '''
        self._scene.remove_attached_object(self._eef_link, name=self._box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True,
                                          box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        '''Removing Objects from the Planning Scene. We can remove the box
        from the world
        :param timeout: timeout in seconds, defaults to 4
        :type timeout: int, optional
        :return: Method to wait until the changes are reflected
        :rtype: KR1410Moveit.wait_for_state_update
        '''
        self._scene.remove_world_object(self._box_name)

        # **Note:** The object must be detached before we can remove it from
        #  the world.
        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False,
                                          box_is_known=False, timeout=timeout)

    def __del__(self):
        '''Destructor Method
        '''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class KR1410Moveit Deleted." + '\033[0m')

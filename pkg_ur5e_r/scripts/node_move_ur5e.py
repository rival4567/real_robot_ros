#!/usr/bin/env python3

import rospy
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import tf2_ros
import tf2_msgs.msg
import math

from schunk_gripper.srv import SchunkGripper


class CartesianPath:
    '''This class moves the ur5 arm using MoveIt!'''

    # Constructor
    def __init__(self):
        # Initialize node anonymously
        rospy.init_node('node_move_ur5e', anonymous=True)

        # Initialize `moveit_commander`_ node
        moveit_commander.roscpp_initialize('ur5e_robot')

        # Kinematic model and the robot's current joint states
        self._robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object. This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self._scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object. This object is an interface
        # to a planning group (group of joints).
        # This interface can be used to plan and execute motions:
        self._planning_group = "manipulator"
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in RViz:
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
            queue_size=1)

        # Creating a simple action client to send goals.
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        # Other class variables
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z, synchronous="True"):
        '''This method use cartesian path to translate in x, y, z direction. It takes 4
        arguments: trans_x, trans_y, trans_z, synchronous. trans x, y, z takes translation
        in metre. synchronous is of bool type which gives movement of arm synchronously 
        (smoother) or async'''

        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(self._group.get_current_pose().pose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        fraction = 0.0
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold

        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan, wait=synchronous)

    def set_joint_angles(self, arg_list_joint_angles):
        '''Setting joint angles for 'ur5_1_planning_group' all links.'''

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        # Move to desired joint angles
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        # Calling `stop()` ensures that there is no residual movement.
        self._group.stop()

        return flag_plan

    def go_to_pose(self, arg_pose):
        '''Go to desired pose for the end-effector'''

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        # Move to target position
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found."
                + '\033[0m')

        # Calling 'stop()' ensures that there is no residual movement.
        self._group.stop()
        # Clearing targets after planning of poses.
        self._group.clear_pose_targets()
        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


class tfEcho:
    '''This class provides tf2 transform between different coordinate frames.'''

    # Constructor
    def __init__(self):
        # Get transform in buffer
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

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

    def calc_tf(self, arg_frame_1="world", arg_frame_2="ur5_wrist_3_link"):
        '''This method finds transform between two coordinate frames and return
        transform. It takes two coordinate frames as input.'''

        try:
            trans = self._tfBuffer.lookup_transform(
                arg_frame_1, arg_frame_2, rospy.Time())

            # Calculate transform
            ref_pos = geometry_msgs.msg.Pose()
            ref_pos.position.x = trans.transform.translation.x
            ref_pos.position.y = trans.transform.translation.y
            ref_pos.position.z = trans.transform.translation.z

            ref_pos.orientation.x = trans.transform.rotation.x
            ref_pos.orientation.y = trans.transform.rotation.y
            ref_pos.orientation.z = trans.transform.rotation.z
            ref_pos.orientation.w = trans.transform.rotation.w

            return ref_pos

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")
            return None

    def ee_world_conv(self, pos):
        '''This method converts End-Effector frame to world and return position
        in world frame.'''
        temp = pos.position.x
        pos.position.x = -pos.position.z
        pos.position.y = temp
        pos.position.z = -pos.position.y

        return pos

    def calc_distance(self, pos1, pos2):
        '''This calculates distance between two position coordinates.'''
        return (pos1.position.x - pos2.position.x, pos1.position.y - pos2.position.y,
                pos1.position.z - pos2.position.z)

    # Destructor
    def __del__(self):
        rospy.loginfo(
            '\033[94m' + "Object of class tfEcho Deleted." + '\033[0m')


def main():
    # Define object for classes
    ur5 = CartesianPath()
    tf = tfEcho()

    pick_ball_srv = rospy.ServiceProxy(
        '/ur5e/connect/GazeboGripper/activate', SchunkGripper)
    pick_ball_srv.wait_for_service()

    ur5e_pick_joint_angles = tf.list_degrees_to_radians(
        [254.79, -100.37, -122.19, -48.98, -270.54, 44.03])
    ur5e_place_joint_angles = tf.list_degrees_to_radians(
        [325.45, -103.26, -116.62, -54.35, -271.28, 119.25])
    ur5e_home_joint_angles = tf.list_degrees_to_radians(
        [90, -90, 0, -90, 0, 90])

    # gripper_srv = rospy.ServiceProxy(
    #     'ur5e/connect/GazeboGripper/activate', SchunkGripper)
    # rospy.wait_for_service(gripper_srv)

    while not rospy.is_shutdown():
        # Infinite loop unless Ctrl + C or some error.

        # Wait for packages to spawn.
        rospy.sleep(1)
        ur5.set_joint_angles(ur5e_pick_joint_angles)
        pick_ball_srv.call(True)
        rospy.sleep(1)
        ur5.ee_cartesian_translation(0, 0, -0.1)
        rospy.sleep(1)
        ur5.set_joint_angles(ur5e_place_joint_angles)
        rospy.sleep(1)
        pick_ball_srv.call(False)
        ur5.ee_cartesian_translation(0, 0, -0.1)
        break


if __name__ == '__main__':
    main()

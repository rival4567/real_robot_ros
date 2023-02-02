#!/usr/bin/env python3
'''
    kr1410TaskActionServer
    =======================
    This will create an action server where goals can be give to
    manipulate kr1410 robot. It will be helpful for ROSLIB library
    for online web interface of this robot.
    *******************************************************************
    Author: Shivam Shivam (shivam.shivam@stud.th-deg.de)
    *******************************************************************
    University: Technologie Hochschule Deggendorf
    ---------------------------------------------
'''

import rospy
import actionlib
import threading

from kr1410_moveit.kr1410 import KR1410Moveit

from control_msgs.msg import FollowJointTrajectoryActionFeedback

from pkg_kr_sim.msg import goPoseAction
from pkg_kr_sim.msg import goPoseActionResult
from pkg_kr_sim.msg import goJointAction
from pkg_kr_sim.msg import goJointActionResult
from pkg_kr_sim.msg import goJointActionFeedback
from pkg_kr_sim.srv import saveTrajectory, saveTrajectoryResponse
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import MoveItErrorCodes

from geometry_msgs.msg import Pose


class KR1410ActionServer():
    '''This class contains methods to move KR1410 Robot using web
     application.
    '''

    def __init__(self):
        '''Constructor Method
        '''
        # Initialize the Action Server
        self._goPoseAS = actionlib.ActionServer('/kr1410/goal_pose',
                                                goPoseAction,
                                                self.on_goal,
                                                self.on_cancel,
                                                auto_start=False)
        '''
        * self.on_goal - It is the fuction pointer which points to a function
        which will be called when the Action Server receives a Goal.
        * self.on_cancel - It is the fuction pointer which points to a function
        which will be called when the Action Server receives a Cancel Request.
        '''

        self._jointAS = actionlib.ActionServer('/kr1410/goal_joint',
                                               goJointAction,
                                               self.on_goal,
                                               self.on_cancel,
                                               auto_start=False)

        # Starting Action Server
        self._goPoseAS.start()
        self._jointAS.start()
        self.thread_list = []
        # To synchonize the threads, we are using lock
        self.lock = threading.Lock()

        rospy.Subscriber("/manipulator_controller/follow_joint_trajectory/feedback",
                         FollowJointTrajectoryActionFeedback, self.manipulator_feedback_sub)

        self.pose_pub = rospy.Publisher(
            '/kr1410/current_pose', Pose, queue_size=10)
        self.manipulator_feedback = goJointActionFeedback()

        self.trajectory_srv = rospy.Service(
            '/kr1410/save_trajectory', saveTrajectory, self.handle_save_trajectory_srv)
        self.temp_goal_handle = None

        rospy.loginfo(
            '\033[92m' + 'Started "Go to Pose" Action Server.' + '\033[0m')

        self.kr1410 = KR1410Moveit('', 'manipulator')
        self.gripper = KR1410Moveit('', 'end_effector')

        # Planner is RRTConnect. Comment to use RRTStar planner.
        self.kr1410.group.set_planning_time(5)
        self.kr1410.group.set_planner_id("RRTConnect")

        self.rate = rospy.Rate(10)

    def handle_save_trajectory_srv(self, req):
        print("Request is", req)
        computed_plan = ''
        joints = []
        pose = Pose()
        trajectories = RobotTrajectory()
        error_codes = MoveItErrorCodes()

        if req.use_joints:
            joints = req.joints
            self.kr1410.group.set_joint_value_target(joints)

        else:
            pose = req.pose
            self.kr1410.group.set_pose_target(pose)

        computed_plan = self.kr1410.group.plan()
        trajectories = computed_plan[1]
        error_codes = computed_plan[3]
        return saveTrajectoryResponse(computed_plan[0], trajectories, computed_plan[2], error_codes)

    def manipulator_feedback_sub(self, msg):
        '''Subscriber to '/manipulator_controller/follow_joint_trajectory/feedback'
        :param msg: message from the rostopic
        :type msg: ROS message type FollowJointTrajectoryActionFeedback()
        '''
        self.manipulator_feedback.feedback = msg.feedback

    def on_goal(self, goal_handle):
        '''Callback to get new goals
        :param goal_handle: Action server works on this
        :type goal_handle: actionlib.action_client.ClientGoalHandle
        '''
        goal = goal_handle.get_goal()
        status = goal_handle.get_goal_status().status
        rospy.loginfo("Received new goal from Client.")
        rospy.loginfo(goal)

        # Validating incoming goal parameters
        if status == 0:
            self.thread_list.append(threading.Thread(
                name="worker", target=self.process_goal, args=(goal_handle,)))
            if not self.thread_list:
                rospy.loginfo("No new goals. Waiting for a new goal!")
            elif len(self.thread_list) == 1:
                rospy.loginfo("New goal received. Starting the goal request.")
            elif len(self.thread_list) > 1:
                rospy.loginfo("New goals received. Goal in pending state.")
        else:
            goal_handle.set_rejected()
            return

    def process_goal(self, goal_handle):
        '''This method accepts new goals in queue.
        :param goal_handle: `goal_handle` on which action server works
        :type goal_handle: actionlib.action_client.ClientGoalHandle
        '''
        self.lock.acquire()
        goal_handle.set_accepted()
        result = goPoseActionResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal: " + str(goal_id.id))

        goal = goal_handle.get_goal()

        self.temp_goal_handle = goal_handle

        planning_group = self.kr1410

        if hasattr(goal, 'controller'):
            # Do end-effector task simulataneously instead of waiting
            if goal.controller == "end_effector":
                planning_group = self.gripper

            target_joints = goal.joint.trajectory.points[0].positions
            rospy.loginfo("Received request for joint control ({}).".format(
                goal.joint.trajectory.points[0]))
            result = planning_group.set_joint_angles(target_joints)

        else:
            target_pose = goal.pose
            rospy.loginfo("Received request to go to Pose ({}, {})".format(
                goal.pose.position, goal.pose.orientation))
            result = self.kr1410.go_to_pose(target_pose)

        print(result)
        # Goal Result
        rospy.loginfo("Send goal result to client.")
        if result:
            rospy.loginfo("Succeeded!")
            goal_handle.set_succeeded()
        else:
            rospy.loginfo("Failed goal. Aborting.")
            goal_handle.set_aborted()

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")
        self.temp_goal_handle = None
        self.lock.release()

    @staticmethod
    def on_cancel(goal_handle):
        '''This function will be called when Goal Cancel request is send to
            the Action Server
        :param goal_handle: `goal_handle` on which Action server works
        :type goal_handle: actionlib.action_client.ClientGoalHandle
        '''
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()
        goal_handle.set_canceled()
        rospy.loginfo(goal_id)

    def __del__(self):
        '''Destructor Method'''
        self._goPoseAS.stop()
        self._jointAS.stop()
        rospy.loginfo(
            '\033[94m' + "Object of class KR1410ActionServer Deleted."
            + '\033[0m')


def main():
    '''Main Function: Start Action Server and initialize node.'''

    # 1. Initialize ROS node
    rospy.init_node('node_kr1410_task_action_server')

    # 2. Create Action Server object
    server = KR1410ActionServer()
    server.kr1410.set_joint_angles([0, 0, 0, 0, 0, 0, 0])
    server.kr1410.save_planned_trajectories([1.57, 0, 0, 0, 0, 0, 0])
    # server.gripper.set_joint_angles([0.8])

    while not rospy.is_shutdown():
        # Infinite loop until Ctrl+C
        server.pose_pub.publish(server.kr1410.group.get_current_pose().pose)
        if len(server.thread_list) >= 1 and (not server.lock.locked()):
            server.thread_list[0].start()
            server.thread_list.pop(0)

        if server.temp_goal_handle:
            server.temp_goal_handle.publish_feedback(
                server.manipulator_feedback)
        server.rate.sleep()
    del server


if __name__ == '__main__':
    main()

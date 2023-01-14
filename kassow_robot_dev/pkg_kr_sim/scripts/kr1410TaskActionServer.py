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

from pkg_kr_sim.msg import goPoseAction
from pkg_kr_sim.msg import goPoseActionResult
from pkg_kr_sim.msg import goJointAction
from pkg_kr_sim.msg import goJointActionResult


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

        rospy.loginfo(
            '\033[92m' + 'Started "Go to Pose" Action Server.' + '\033[0m')

        self.kr1410 = KR1410Moveit('', 'manipulator')
        self.gripper = KR1410Moveit('', 'end_effector')

        # Planner is RRTConnect. Comment to use RRTStar planner.
        self.kr1410.group.set_planning_time(5)
        self.kr1410.group.set_planner_id("RRTConnect")

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

        planning_group = self.kr1410

        if hasattr(goal, 'controller'):
            # Do end-effector task simulataneously instead of waiting
            if goal.controller == "end_effector":
                planning_group = self.gripper

            target_joints = goal.joint.goal.trajectory.points[0].positions
            rospy.logwarn("Received request for joint control ({}).".format(
                goal.joint.goal.trajectory.points[0]))
            planning_group.set_joint_angles(target_joints)

        else:
            print("AKJLFLAKFAKJFE213131", goal.pose)
            target_pose = goal.pose
            rospy.logwarn("Received request to go to Pose ({}, {})".format(
                goal.pose.position, goal.pose.orientation))
            self.kr1410.go_to_pose(target_pose)

        # Goal Result
        rospy.loginfo("Send goal result to client.")
        if result:
            rospy.loginfo("Succeeded!")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Failed goal. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")
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
    server.gripper.set_joint_angles([0.8])
    rospy.sleep(1)

    while not rospy.is_shutdown():
        # Infinite loop until Ctrl+C
        if len(server.thread_list) >= 1 and (not server.lock.locked()):
            server.thread_list[0].start()
            print("AAKJDADKJAJD ", len(server.thread_list), server.thread_list)
            server.thread_list.pop(0)
            print("flajfkakfjlkaf ", len(server.thread_list), server.thread_list)

    del server


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rospy

from schunk_gripper.srv import SchunkGripper
from schunk_gripper.srv import SchunkGripperResponse

from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelState


class GazeboGripperWork(object):

    def __init__(self) -> None:

        self.link_name = LinkStates().name
        self.link_pose = LinkStates().pose
        self.attachable = False
        self.attached = False
        self.rate = rospy.Rate(10)  # Rate in Hz
        self.model_state = ModelState()

        self.link_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates,
                                                self.link_states_callback)

        self.model_publisher = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=10)

    def link_states_callback(self, joint_states):
        self.link_name = joint_states.name
        self.link_pose = joint_states.pose
        wrist_3_index = self.link_name.index('robot::wrist_3_link')
        ball_index = self.link_name.index('cricket_ball::link')
        wrist_3_link_pose = self.link_pose[wrist_3_index]
        ball_link_pose = self.link_pose[ball_index]

        self.model_state.pose = wrist_3_link_pose
        self.model_state.pose.position.x = wrist_3_link_pose.position.x
        self.model_state.pose.position.y = wrist_3_link_pose.position.y - 0.005
        self.model_state.pose.position.z = wrist_3_link_pose.position.z - 0.12
        self.model_state.pose.orientation = wrist_3_link_pose.orientation
        self.model_state.reference_frame = "world"
        self.model_publisher.publish(self.model_state)

        # Check if in range to be attachable
        self.attachable = self.check_range(wrist_3_link_pose, ball_link_pose)

    def check_range(self, wrist_3_link_pose, ball_link_pose):

        if abs(wrist_3_link_pose.position.x - ball_link_pose.position.x) < 0.005 and \
            abs(wrist_3_link_pose.position.y - ball_link_pose.position.y) < 0.05 and \
                abs(wrist_3_link_pose.position.z - ball_link_pose.position.z) < 0.2 and \
                not self.attached:
            rospy.loginfo(
                '\033[94m' + " >>> Attachable object found!" + '\033[0m')
            return True

        return False

    def close_gazebo_gripper(self):

        if self.attachable:
            self.model_state.model_name = "cricket_ball"
            rospy.loginfo(
                '\033[94m' + " >>> Object Attached!" + '\033[0m')
            self.attached = True
            return True

        rospy.logwarn(" >>> Object couldn't be picked up!")
        return False

    def open_gazebo_gripper(self):

        if self.attached:
            self.model_state.model_name = ""
            rospy.loginfo(
                '\033[94m' + " >>> Object Detached!" + '\033[0m')
            self.attached = False
            return True
        rospy.logwarn(" >>> Object couldn't be detached!")
        return False

    def callback_service_on_request(self, req):

        rospy.loginfo('\033[94m' + " >>> Schunk Gripper Activate: " +
                      str(req.activate_schunk_gripper) + '\033[0m')

        if req.activate_schunk_gripper:
            success = self.close_gazebo_gripper()
            if success:
                return SchunkGripperResponse(True)
            rospy.logerr("Unable to connect to UR5e IO...")
        else:
            success = self.open_gazebo_gripper()
            if success:
                return SchunkGripperResponse(False)


def main():
    '''Initialise ROS node and start gripper service.'''

    rospy.init_node('node_service_gazebo_gripper')

    gazebo_gripper = GazeboGripperWork()

    rospy.Service('/ur5e/connect/GazeboGripper/activate', SchunkGripper,
                  gazebo_gripper.callback_service_on_request)

    rospy.loginfo(" >>> Schunk Gripper Service Server Ready.")

    rospy.spin()


if __name__ == '__main__':
    main()

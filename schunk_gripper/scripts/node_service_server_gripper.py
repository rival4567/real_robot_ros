#!/usr/bin/env python3

import rospy

from ur_msgs.srv import SetIO
from ur_msgs.srv import SetIORequest


from schunk_gripper.srv import SchunkGripper
from schunk_gripper.srv import SchunkGripperResponse


class SchunkGripperWork(object):

    def __init__(self) -> None:
        self.set_io_req = SetIORequest()
        self.set_io_req.fun = 1
        rospy.loginfo("Waiting for UR5e IO service...")
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        try:
            self.set_ur5e_pin_srv = rospy.ServiceProxy(
                '/ur_hardware_interface/set_io', SetIO)
            rospy.loginfo("Connected to UR5e IO Service!")
            pin16 = self.set_ur5e_pin_srv()
        except rospy.ServiceException as _:
            print("Service call failed: " + str(_))

    def close_schunk_gripper(self):

        self.set_io_req.pin = 16
        self.set_io_req.state = 0.0
        pin16_response = self.set_ur5e_pin_srv.call(self.set_io_req)
        rospy.sleep(0.5)

        self.set_io_req.pin = 17
        self.set_io_req.state = 1.0
        pin17_response = self.set_ur5e_pin_srv.call(self.set_io_req)
        rospy.sleep(0.5)

        return pin16_response.success and pin17_response.success

    def open_schunk_gripper(self):

        self.set_io_req.pin = 16
        self.set_io_req.state = 1.0
        pin16_response = self.set_ur5e_pin_srv.call(self.set_io_req)
        rospy.sleep(0.5)

        self.set_io_req.pin = 17
        self.set_io_req.state = 0.0
        pin17_response = self.set_ur5e_pin_srv.call(self.set_io_req)
        rospy.sleep(0.5)

        return pin16_response.success and pin17_response.success

    def callback_service_on_request(self, req):
        rospy.loginfo('\033[94m' + " >>> Schunk Gripper Activate: " +
                      str(req.activate_schunk_gripper) + '\033[0m')

        if req.activate_schunk_gripper:
            success = self.close_schunk_gripper()
            if success:
                return SchunkGripperResponse(True)
            rospy.logerr("Unable to connect to UR5e IO...")
        else:
            success = self.open_schunk_gripper()
            if success:
                return SchunkGripperResponse(False)


def main():
    '''Initialise ROS node and start gripper service.'''

    rospy.init_node('node_service_schunk_gripper')

    schunk_gripper = SchunkGripperWork()

    rospy.Service('/ur5e/connect/SchunkGripper/activate', SchunkGripper,
                  schunk_gripper.callback_service_on_request)

    rospy.loginfo(" >>> Schunk Gripper Service Server Ready.")

    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import argparse
import rospy
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class DisplayTrajectoryLine():
    '''This class will display trajectory line on the web server.'''
    def __init__(self):
        '''Constructor Method'''
        rospy.wait_for_service('compute_fk')
        self.compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        parser = argparse.ArgumentParser()
        parser.add_argument('-fk_link_name', type=str, metavar='FK_LINK_NAME', help='link name for the trajectory to be published',required=True)
        args = rospy.myargv()
        self.args = parser.parse_args(args[1:])
        self.fk_link = self.args.fk_link_name
        self.trajectory_sub = rospy.Subscriber('move_group/display_planned_path', DisplayTrajectory, self.trajectory_cb)
        self.trajectory_pub = rospy.Publisher('kr1410/trajectory_line', Path, queue_size=10)
        self.get_position_fk = GetPositionFKRequest()
        self.display_trajectory = DisplayTrajectory()
        self.path = Path()
        self.pose_stamped = []

    def trajectory_cb(self, msg):
        '''Callback to the topic /move_group/display_planned_path'''
        self.pose_stamped = []
        self.display_trajectory = msg
        self.get_position_fk.fk_link_names = [self.fk_link]
        self.get_position_fk.robot_state.joint_state.name = self.display_trajectory.trajectory[0].joint_trajectory.joint_names
        for point in self.display_trajectory.trajectory[0].joint_trajectory.points:
            self.get_position_fk.robot_state.joint_state.position = point.positions
            self.get_position_fk.robot_state.joint_state.velocity = point.velocities
            self.get_position_fk.robot_state.joint_state.effort = point.effort
            response = self.compute_fk.call(self.get_position_fk)
            if(response.error_code.val == 1):
                self.pose_stamped.append(response.pose_stamped[0])
        self.path.poses = self.pose_stamped
        self.path.header = self.pose_stamped[0].header

    def __del__(self):
        '''Destructor Method'''
        self.trajectory_sub.unregister()
        rospy.logwarn('Shutting down the trajectory_line node!')

def main():
    rospy.init_node('kr1410_trajectory_line', anonymous=False)
    rate = rospy.Rate(60) #60Hz
    # Wait to load all the nodes
    rospy.sleep(10)
    dtl = DisplayTrajectoryLine()
    rospy.loginfo(">>> Published planned trajectories. Press Ctrl+C to shutdown! <<<")

    while not rospy.is_shutdown():  
        if dtl.display_trajectory.trajectory and dtl.path.poses:    
            dtl.trajectory_pub.publish(dtl.path)
        rate.sleep()

if __name__ == '__main__':
    main()
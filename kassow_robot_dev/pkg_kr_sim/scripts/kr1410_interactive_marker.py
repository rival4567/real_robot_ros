#!/usr/bin/env python3
import rospy

from visualization_msgs.msg import InteractiveMarkerUpdate

IM_UPDATE = InteractiveMarkerUpdate()


def interactive_marker_callback(msg):
    global IM_UPDATE
    IM_UPDATE = msg
    if len(IM_UPDATE.poses) > 0:
        for pose in IM_UPDATE.poses:
            pose.name = "tool_io"
    rospy.loginfo(IM_UPDATE)

# def handle_init_marker_srv(srv):
#     pass


def main():
    '''Main Function'''

    # 1. Initialize ROS node
    rospy.init_node('node_kr1410_interactive_marker')

    topic = '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/'

    # 2. Subscribe to RViz Interactive marker from MoveIt!
    rospy.Subscriber(topic + "update", InteractiveMarkerUpdate,
                     interactive_marker_callback)

    # rospy.Service(topic + 'tunneled/get_init',
    #               InteractiveMarkerUpdate, handle_init_marker_srv)

    pub = rospy.Publisher(topic + 'tunneled/update',
                          InteractiveMarkerUpdate, queue_size=10)

    rate = rospy.Rate(10)
    # 3. Keep running the node
    while not rospy.is_shutdown():
        pub.publish(IM_UPDATE)
        rate.sleep()


if __name__ == '__main__':
    main()

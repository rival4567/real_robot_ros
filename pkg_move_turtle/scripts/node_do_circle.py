#!/usr/bin/env python3

from math import pi, sqrt
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import rospy


class turtle:
    '''This class has variables and method to manipulate turtle 
    movement.
    '''

    def __init__(self):
        # Creates a node with name 'node_turtle_revolve' and make
        # sure it is a unique node (using anonymous=True)
        rospy.init_node('node_turtle_revolve', anonymous=True)

        # A subscriber to the topic '/turtle1/pose'. 'self.callback'
        # is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.callback)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        self.pose = Pose()
        self.anticlockwise = True
        self.rotation = 0
        self.previous_theta = 0
        self.rate = rospy.Rate(10)  # Rate in Hz

    def callback(self, pos_msg):
        '''Callback function which is called when a new message of type 
        Pose is received by the subscriber.
        '''
        self.pose = pos_msg
        current_theta = self.pose.theta

        # Changes (-pi, pi) to (0, 2*pi)
        self.pose.theta += 2*pi if pos_msg.theta < 0 else 0
        self.pose.theta += 2*pi*self.rotation

        # Counting number of revolutions and updating self.rotation
        # based on anticlockwise or clockwise rotation
        if self.anticlockwise:
            if current_theta > 0 and self.previous_theta < 0:
                self.rotation += 1
        else:
            if current_theta < 0 and self.previous_theta > 0:
                self.rotation += 1
        self.previous_theta = current_theta

    def getDistance(self, goal_pose):
        ''' This method calculates the Euclidean distance.'''
        return sqrt(pow(goal_pose.x - self.pose.x, 2)
                    + pow(goal_pose.x - self.pose.y, 2))

    def traceCircle(self, turns, velocity, anticlockwise=True):
        '''Turtle traces a circle. This method takes 3 arguments.
        'turns', 'velocity', 'anticlockwise'. Turns gives number of 
        circular rotations. 'velocity' is of tuple data type (linear 
        velocity, angular velocity). 'anticlockwise' is of boolean
        data type which gives direction of rotation.
        '''
        self.anticlockwise = anticlockwise
        vel_msg = Twist()

        # Waiting for self.pose to be updated and captured.
        # First few values are 0.
        while True:
            if self.pose.x == 0 or self.pose.y == 0:
                continue
            else:
                goal_pose = self.pose
                break

        constant = 1

        while not rospy.is_shutdown():
            # Publishes to '/turtle1/cmd_vel1'. This makes turtle
            # move in circle and stop when circle is complete.

            # To provide deceleration or acceleration to turtle. Use if
            # turtle is not out of goal position or frequency is too
            # low ~1Hz. For normal operation keep 'constant=1'.
            # theta = 2*pi - self.pose.theta
            # constant = 0.5 if theta < 3.14 else 1
            (lin_v, ang_v) = (abs(velocity[0])*constant,
                              abs(velocity[1])*constant)

            if self.pose.theta <= 2*pi*turns:
                vel_msg.linear.x = abs(lin_v)
                vel_msg.angular.z = ang_v if anticlockwise else -ang_v

                # Publishing at desired rate
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
                rospy.loginfo('Moving in a circle, Angle: '
                              + str(self.pose.theta)
                              + ', X: ' + str(self.pose.x)
                                + ', Y: ' + str(self.pose.y))
            else:
                # Stopping the turtle when goal is reached.
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
                rospy.loginfo('Goal is reached, angle offset: '
                              + str(self.pose.theta - 2*pi*turns)
                              + ', X offset: ' + str(self.pose.x
                                                     - goal_pose.x)
                              + ', Y offset: ' + str(self.pose.y
                                                     - goal_pose.y))

                # If we press control + C, the node will stop.
                rospy.spin()


if __name__ == '__main__':
    try:
        turns = 1   # Number of revolution(s) the turtle should make
        linear_velocity = 0.75
        angular_velocity = 0.75
        velocity = (linear_velocity, angular_velocity)

        # Set pen of turtle1
        set_t1_pen_srv = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        set_t1_pen_srv.wait_for_service()
        set_t1_pen_srv.call(255, 255, 0, 5, 0)  # yellow

        t = turtle()
        # Making sure velocity is not equal to zero or different.
        # Provide anticlockwise bool value for direction of
        if not (linear_velocity == 0.0 or angular_velocity == 0.0 and
                not linear_velocity == angular_velocity):
            t.traceCircle(turns, velocity, True)

    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated.')

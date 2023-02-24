# !/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from math import pow, atan2, sqrt


class Follower_Controller:
    def __init__(self):
        # Creates a node with name 'Position_Controller_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('controller')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pub_neg_x = rospy.Subscriber('loadcell_neg_x', Int32, self.get_neg_x)
        self.pub_pos_x = rospy.Subscriber('loadcell_pos_x', Int32, self.get_pos_x)
        self.pub_y = rospy.Subscriber('loadcell_y', Int32, self.get_y)

        self.neg_force_x = 0
        self.pos_force_x = 0
        self.force_y = 0
        self.rate = rospy.Rate(10)

    def get_neg_x(self, data):
        self.neg_force_x = data

    def get_pos_x(self, data):
        self.neg_force_x = data

    def get_y(self, data):
        self.force_y = data

    def minimiseForce(self):

        vel_msg = Twist()

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control
        p = 0.01


        x_force = self.pos_force_x - self.neg_force_x
        # Set Linear velocity in the x-axis.
        vel_msg.linear.x = x_force*p
        vel_msg.linear.y = self.force_y*p
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)

        # Publish at the desired rate.
        self.rate.sleep()

        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        follower_controller = Follower_Controller()
        x.minimiseForce()
    except rospy.ROSInterruptException:
        pass

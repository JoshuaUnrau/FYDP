#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, TransformStamped
import math

class Angle_Testing:
    def __init__(self):
        rospy.init_node("angle_testing")
        self.goal_subscriber = rospy.Subscriber('vicon/test_table_leg_1/test_table_leg_1', TransformStamped,
                                                self.set_goal)
        # test topic for testing leg positions
        self.pose_subscriber = rospy.Subscriber('/vicon/test/test', TransformStamped, self.update_pose)

        # subscribers for table leg positions
        self.table_leg_1_subscriber = rospy.Subscriber('vicon/test_table_leg_1/test_table_leg_1', TransformStamped,
                                                       self.set_table_leg_1_pose)
        self.table_leg_2_subscriber = rospy.Subscriber('vicon/test_table_leg_2/test_table_leg_2', TransformStamped,
                                                       self.set_table_leg_2_pose)

        # position of the robot
        # self.pose_subscriber = rospy.Subscriber('/vicon/swole_1/swole_1', TransformStamped, self.update_pose)
        self.pose_subscriber = rospy.Subscriber('/vicon/test/test', TransformStamped, self.update_pose)


if __name == '__main__':
    try:
        x = Angle_Testing()
    except rospy.ROSInterruptException:
        pass
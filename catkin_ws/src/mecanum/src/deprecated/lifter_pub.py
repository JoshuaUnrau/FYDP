#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from enum import IntEnum
from std_msgs.msg import Float32

class Lifter_States(IntEnum):
    neutral = 0
    up = 1
    down = 2

def lifter_pub():
    pub = rospy.Publisher('desired_lifter_state', Int32, queue_size=10)
    rospy.init_node('lifter_test', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    lifter_state_test = Int32()
    lifter_state_test = Lifter_States.up

    # vel_msg.linear.y = 5
    while not rospy.is_shutdown():
        # test_vel = 1
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo("Testing vel", vel_msg)
        print("Test lifter state", int(lifter_state_test))
        pub.publish(lifter_state_test)
        rate.sleep()

if __name__ == '__main__':
    try:
        lifter_pub()
    except rospy.ROSInterruptException:
        pass


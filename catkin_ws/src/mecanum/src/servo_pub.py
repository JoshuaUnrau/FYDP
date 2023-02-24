#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from enum import IntEnum

class Gripper_States(IntEnum):
    closed = 0
    open = 1

def servo_pub():
    pub = rospy.Publisher('servo', Int32, queue_size=10)
    rospy.init_node('servo_test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    gripper_state_test = Int32()
    gripper_state_test =  20

    while not rospy.is_shutdown():
        # test_vel = 1
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo("Testing vel", vel_msg)
        print("Test Gripper state", gripper_state_test)
        pub.publish(gripper_state_test)
        rate.sleep()

if __name__ == '__main__':
    try:
        servo_pub()
    except rospy.ROSInterruptException:
        pass


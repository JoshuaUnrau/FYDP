#!/usr/bin/env python

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def motor_test():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('motor_test', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    vel_msg = Twist()
   # vel_msg.linear.x = -1
    # vel_msg.linear.y = 5
    while not rospy.is_shutdown():
        # test_vel = 1
        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo("Testing vel", vel_msg)
        print("Test vel", vel_msg)
      #  pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        motor_test()
    except rospy.ROSInterruptException:
        pass

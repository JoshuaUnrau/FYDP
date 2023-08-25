#!/usr/bin/env python3

#TODO:
#1. Get desired angles relative to table for gripping positions
#2.

import rospy
from std_msgs.msg import Float64, Float32, Int32, Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, TransformStamped, Point
from math import pow, atan2, sqrt
from enum import IntEnum
import math
import time
from timeit import default_timer as timer

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

class RobotState(IntEnum):
    Stabilise = 1
    Initialise = 0

class Controller:

    def __init__(self):
        # Creates a node with name 'Position_Controller_controller' and make sure it is a
        # unique node (using anonymous=True).
        print("Initialising")
        rospy.init_node('controller')

        self.maximum_pwm = 60
        self.minimum_pwm = 10

        self.orientation = Point()
        self.goal_orientation = Point()

        self.shutdown = True
        self.direct_control = True

        self.pub_motor_front_left = rospy.Publisher('motor/front/left', Float32, queue_size=5)
        self.pub_motor_front_right = rospy.Publisher('motor/front/right', Float32, queue_size=5)
        self.pub_motor_rear_left = rospy.Publisher('motor/rear/left', Float32, queue_size=5)
        self.pub_motor_rear_right = rospy.Publisher('motor/rear/right', Float32, queue_size=5)

        # test topic for testing leg positions
        self.orientation_subscriber = rospy.Subscriber('orientation', TransformStamped, self.update_pose)
        self.shutdown_sub = rospy.Subscriber('shutdown_topic_name', Bool, self.shutdown_callback)
        self.direct_control_sub = rospy.Subscriber('direct_control_topic_name', Bool, self.direct_control_callback)
        self.manageState()

    def shutdown_callback(self, data):
        self.shutdown = data.data
        print(f"Shutdown received: {self.shutdown}")

    def direct_control_callback(self, data):
        self.direct_control = data.data
        print(f"Direct control received: {self.direct_control}")

    def update_pose(self, data):
        self.orientation.x = data.x
        self.orientation.y = data.y
        self.orientation.z = data.z

    def manageState(self):
        self.rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if not self.direct_control and not self.shutdown:
                self.state_stabilise()
            self.rate.sleep()

    def state_startup(self):
        self.servo_publisher.publish(self.SERVO_OPEN)
        self.lifter_publisher.publish(ActuatorState.Down)

    def state_stabilise(self):
        self.goal_orientation = Point()
        self.goal_orientation.x = 0 #Roll
        self.goal_orientation.y = 0 #Pitch
        self.goal_orientation.z = 0 #Yaw

        error = Point()
        error.x = self.goal_orientation.x - self.orientation.x
        error.y = self.goal_orientation.y - self.orientation.y
        error.z = self.goal_orientation.z - self.orientation.z

        p = 0.3
        base = 20

        #Roll PID
        self.pub_motor_front_left.publish(clamp(error.x * p + base, 20, 80))
        self.pub_motor_front_right.publish(clamp(error.x * p + base, 20, 80))
        self.pub_motor_rear_left.publish(clamp(-error.x * p + base, 20, 80))
        self.pub_motor_rear_right.publish(clamp(-error.x * p + base, 20, 80))

def clamp(n, min, max):
    if n < min:
        return min
    elif n > max:
        return max
    else:
        return n

if __name__ == '__main__':
    try:
        x = Controller()
    except rospy.ROSInterruptException:
        pass

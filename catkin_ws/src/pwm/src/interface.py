#!/usr/bin/env python3
import rospy
import os
import sys
import time
import random
from multiprocessing import Queue
from webserver import DroneDashboard
from xbox import Joystick

from geometry_msgs.msg import Pose, TransformStamped, Point
from std_msgs.msg import Float64, Float32, Int32, Bool
from multiprocessing import Process
import atexit
import threading
import signal

#https://github.com/niru-5/imusensor
class Interface:
    def __init__(self):

        self.currTime = time.time()
        self.data_queue = Queue()
        self.rate = 100

        # Subscribers for ROS topics
        self.motor_front_left_sub = rospy.Subscriber('motor/front/left', Float32, self.update_motor_front_left)
        self.motor_front_right_sub = rospy.Subscriber('motor/front/right', Float32, self.update_motor_front_right)
        self.motor_rear_left_sub = rospy.Subscriber('motor/rear/left', Float32, self.update_motor_rear_left)
        self.motor_rear_right_sub = rospy.Subscriber('motor/rear/right', Float32, self.update_motor_rear_right)

        self.accel_subscriber = rospy.Subscriber('accel', Point, self.update_accel)
        self.angular_subscriber = rospy.Subscriber('angular', Point, self.update_angular)
        self.magnetometer_subscriber = rospy.Subscriber('magnet', Point, self.update_magnetometer)
        self.orientation_subscriber = rospy.Subscriber('orientation', Point, self.update_orientation)
        self.altitude_subscriber = rospy.Subscriber('altitude', Int32, self.update_altitude)

        # Initialize class attributes
        self.throttle_front_left = 0
        self.throttle_front_right = 0
        self.throttle_rear_left = 0
        self.throttle_rear_right = 0
        self.P, self.I, self.D = 0, 0, 0
        self.acceleration_x, self.acceleration_y, self.acceleration_z = 0, 0, 0
        self.angular_x, self.angular_y, self.angular_z = 0, 0, 0
        self.magnetometer_x, self.magnetometer_y, self.magnetometer_z = 0, 0, 0
        self.orientation_x, self.orientation_y, self.orientation_z = 0, 0, 0
        self.altitude_data = 0

        self.Joystick = Joystick()
        self.pitch_pub = rospy.Publisher("/pitch", Float64, queue_size=10)
        self.roll_pub = rospy.Publisher("/roll", Float64, queue_size=10)
        self.yaw_pub = rospy.Publisher("/yaw", Float64, queue_size=10)
        self.throttle_pub = rospy.Publisher("/throttle", Float64, queue_size=10)
        self.P_pub = rospy.Publisher("/P", Float64, queue_size=10)
        self.I_pub = rospy.Publisher("/I", Float64, queue_size=10)
        self.D_pub = rospy.Publisher("/D", Float64, queue_size=10)
        self.shutdown_pub = rospy.Publisher("/shutdown_signal", Bool, queue_size=10)
        self.direct_control_pub = rospy.Publisher("/direct_control", Bool, queue_size=10)
        self.pub_motor_front_left = rospy.Publisher('motor/front/left', Float32, queue_size=5)
        self.pub_motor_front_right = rospy.Publisher('motor/front/right', Float32, queue_size=5)
        self.pub_motor_rear_left = rospy.Publisher('motor/rear/left', Float32, queue_size=5)
        self.pub_motor_rear_right = rospy.Publisher('motor/rear/right', Float32, queue_size=5)

        self.desired_pitch = self.Joystick.pitch
        self.desired_roll = self.Joystick.roll
        self.desired_yaw = self.Joystick.yaw
        self.throttle = self.Joystick.throttle
        self.shutdown = self.Joystick.shutdown
        self.direct_control = self.Joystick.direct_control

        self.runDashboard = True
        self.dashboard_process = Process(target=self.start_dashboard)
        self.dashboard_process.start()

        self.getReadings()

    def get_joystick(self):
        self.desired_pitch = self.Joystick.pitch
        self.desired_roll = self.Joystick.roll
        self.desired_yaw = self.Joystick.yaw
        self.throttle = self.Joystick.throttle
        self.P += self.Joystick.P*0.01
        self.I += self.Joystick.I*0.01
        self.D += self.Joystick.D*0.01
        self.shutdown = self.Joystick.shutdown
        self.direct_control = self.Joystick.direct_control

        self.shutdown_pub.publish(self.shutdown)
        self.direct_control_pub.publish(self.direct_control)
        self.pitch_pub.publish(self.desired_pitch)
        self.roll_pub.publish(self.desired_roll)
        self.yaw_pub.publish(self.desired_yaw)
        self.throttle_pub.publish(self.throttle)
        self.P_pub.publish(self.P)
        self.I_pub.publish(self.I)
        self.D_pub.publish(self.D)

        if(self.direct_control):
            self.pub_motor_front_left.publish(self.fl_motor)
            self.pub_motor_front_right.publish(self.fr_motor)
            self.pub_motor_rear_left.publish(self.rl_motor)
            self.pub_motor_rear_right.publish(self.rr_motor)

    def start_dashboard(self):
        print("starting dashboard")
        dashboard = DroneDashboard('10.0.0.239', 8050)

        # Start a separate thread for handling the data updates
        update_thread = threading.Thread(target=self.update_dashboard, args=(dashboard, self.data_queue))
        update_thread.start()

        dashboard.run()

    def update_dashboard(self, dashboard, queue):
        last_data_time = time.time()  # Initialize to current time at start
        while self.runDashboard:
            #print("Queue Size: " + str(self.data_queue.qsize()))
            if not queue.empty():
                data = queue.get()
                dashboard.update_data(data)
                last_data_time = time.time()  # Reset the timer when data is received
            elif time.time() - last_data_time > 5:  # 5 seconds without data
                print("No data for 5 seconds. Terminating dashboard.")
                dashboard.terminate_dashboard()
                break
            time.sleep(1 / (2 * self.rate))
        else:
            dashboard.terminate_dashboard()

    def update_motor_front_left(self, data):
        self.throttle_front_left = data.data

    def update_motor_front_right(self, data):
        self.throttle_front_right = data.data

    def update_motor_rear_left(self, data):
        self.throttle_rear_left = data.data

    def update_motor_rear_right(self, data):
        self.throttle_rear_right = data.data

    def update_accel(self, data):
        self.acceleration_x = data.x
        self.acceleration_y = data.y
        self.acceleration_z = data.z

    def update_angular(self, data):
        self.angular_x = data.x
        self.angular_y = data.y
        self.angular_z = data.z

    def update_magnetometer(self, data):
        self.magnetometer_x = data.x
        self.magnetometer_y = data.y
        self.magnetometer_z = data.z

    def update_orientation(self, data):
        self.orientation_x = data.x
        self.orientation_y = data.y
        self.orientation_z = data.z

    def update_altitude(self, data):
        self.altitude_data = data.data

    def normalize_vector(self, vector):
        # Normalize your vector here as required and return the normalized vector
        # Placeholder code (modify as necessary)
        magnitude = sum(x ** 2 for x in vector) ** 0.5
        return [x / magnitude for x in vector] if magnitude else vector

    def point_to_array(self, point):
        # Convert a point to an array (or tuple) here as necessary
        # Placeholder code (modify as necessary)
        return (point.x, point.y, point.z)

    def append_graph_to_dict(self, data_dict):
        return {k + '_graph': v for k, v in data_dict.items()}

    def getReadings(self):
        self.rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.get_joystick()

            # Convert the internal state to a dictionary
            data_dict = {
                "throttle_front_left": self.throttle_front_left,
                "throttle_front_right": self.throttle_front_right,
                "throttle_rear_left": self.throttle_rear_left,
                "throttle_rear_right": self.throttle_rear_right,
                "P": self.P,#self.P,
                "I": self.I,
                "D": self.D,
                "acceleration": (self.acceleration_x, self.acceleration_y, self.acceleration_z),
                "angular": (self.angular_x, self.angular_y, self.angular_z),
                "magnetometer": (self.magnetometer_x, self.magnetometer_y, self.magnetometer_z),
                "orientation": (self.orientation_x, self.orientation_y, self.orientation_z),
                "altitude": self.altitude_data,
                "desired_pitch": self.Joystick.pitch,
                "desired_roll": self.Joystick.roll,
                "desired_yaw": self.Joystick.yaw,
                "throttle": self.Joystick.throttle
            }

           # print(self.Joystick.pitch)

            data_dict = self.append_graph_to_dict(data_dict)

            if self.data_queue.qsize() >= 10:
                self.data_queue.get()
            self.data_queue.put(data_dict)

if __name__ == '__main__':
    try:
        rospy.init_node('Interface')
        rospy.loginfo("Initialising Interface node")
        interface = Interface()
    except rospy.ROSInterruptException:
        #self.dashboard_process.start()
        pass

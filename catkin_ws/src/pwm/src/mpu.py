#!/usr/bin/env python3
import rospy
import os
import sys
import time
import smbus
from multiprocessing import Queue

from webserver import DroneDashboard

from geometry_msgs.msg import Pose, TransformStamped, Point
from std_msgs.msg import Float64, Int32
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman

from multiprocessing import Process
import atexit
import threading

#https://github.com/niru-5/imusensor
class MPU_Driver:
    def __init__(self):
        self.address = 0x68
        self.bus = smbus.SMBus(1)
        self.imu = MPU9250.MPU9250(self.bus, self.address)
        self.connect_mpu()
        self.sensorfusion = kalman.Kalman()
        self.Acceleration = rospy.Publisher('accel', Point, queue_size=10)
        self.Angular = rospy.Publisher('angular', Point, queue_size=10)
        self.Magnetometer = rospy.Publisher('magnet', Point, queue_size=10)
        self.Orientation = rospy.Publisher('orientation', Point, queue_size=10)
        self.Altitude = rospy.Publisher('altitude', Int32, queue_size=10)
        self.currTime = time.time()
        self.getReadings()

    def connect_mpu(self):
        self.imu.begin()

    def getReadings(self):
        self.rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                self.imu.readSensor()
                self.imu.computeOrientation()
                self.sensorfusion.roll = self.imu.roll
                self.sensorfusion.pitch = self.imu.pitch
                self.sensorfusion.yaw = self.imu.yaw
                newTime = time.time()
                dt = newTime - self.currTime
                self.currTime = newTime
                self.sensorfusion.computeAndUpdateRollPitchYaw(self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                                                          self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2], \
                                                          self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2], dt)
                orientation = Point()
                orientation.x = self.imu.roll
                orientation.y = self.imu.pitch
                orientation.z = self.imu.yaw
                self.Orientation.publish(orientation)
            except:
                try:
                    self.connect_mpu()
                except:
                    print("Cannot connect imu")

    def point_to_array(self, point_msg):
        return [point_msg.x, point_msg.y, point_msg.z]

    #TODO: Pass to filter
    def calculate_orientation(self):
        pass

    # TODO: Pass to filter
    def calculate_altitude(self):
        pass

    def normalize_vector(self, v):
        magnitude = (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5
        if magnitude == 0:
            return [0, 0, 0]
        return [v[0] / magnitude, v[1] / magnitude, v[2] / magnitude]


if __name__ == '__main__':
    try:
        rospy.init_node('mpu')
        rospy.loginfo("Initialising mpu node")
        mpu_driver = MPU_Driver()
    except rospy.ROSInterruptException:
        #self.dashboard_process.start()
        pass

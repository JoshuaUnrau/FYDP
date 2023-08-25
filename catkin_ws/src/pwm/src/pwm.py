#!/usr/bin/env python3

import board
import digitalio
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit
import time
import rospy
import math
from enum import IntEnum
from std_msgs.msg import Float64, Float32, Int32, Bool

# in 1 is forward
# in 2 is backwards on motor driver

# actually no the motors are wired inconsistently
# AB pins around the motors are wired randomly

class Motor_Channel(IntEnum):
	front_right = 12
	front_left = 13
	rear_right = 14
	rear_left = 15

class PWM_Driver:
	def __init__(self):
		self.pwm_board_connection_state_pub = rospy.Publisher('pwm_board_connection_state', Int32, queue_size=1)
		self.pwm_board_connection_state_pub.publish(0) #"not connected"
		# Try to great a Digital input
		self.pin = digitalio.DigitalInOut(board.D4)
		print("Digital IO ok!")

		self.i2c = busio.I2C(board.SCL, board.SDA)
		print("I2C ok!")

		self.kit = ServoKit(channels=16, i2c=self.i2c)

		self.shutdown = True
		self.direct_control = True
		self.shutdown_sub = rospy.Subscriber('shutdown_topic_name', Bool, self.shutdown_callback)
		self.direct_control_sub = rospy.Subscriber('direct_control_topic_name', Bool, self.direct_control_callback)

		print("Connecting to PWM")
		self.pca = adafruit_pca9685.PCA9685(self.i2c)
		self.pca.frequency = 60
		self.pwm_board_connection_state_pub.publish(1) #"connected"
		print("Connected to pwm")

	def shutdown_callback(self, data):
		self.shutdown = data.data
		if self.shutdown:
			self.set_motors(0, Motor_Channel.rear_left)
			self.set_motors(0, Motor_Channel.rear_right)
			self.set_motors(0, Motor_Channel.front_left)
			self.set_motors(0, Motor_Channel.front_right)
		print(f"Shutdown received: {self.shutdown}")

	def direct_control_callback(self, data):
		self.direct_control = data.data
		print(f"Direct control received: {self.direct_control}")

	#Maps speed from 0-100 to the pwm value
	def speed_to_pwm(self, speed):
		max_speed = 100
		pwm_max = 65535 #pwm max value
		speed_fraction = speed/max_speed
		if speed > max_speed:
			print("Trying to go faster than max speed")
			return pwm_max

		return speed_fraction*pwm_max

	#Speed is given in m/s
	def set_motors(self, speed, channel_num):
		channel = self.pca.channels[int(channel_num)]

		if self.shutdown:
			channel.duty_cycle = 0
			return
		# print("HERE")
		# print(speed)
		# print(speed.data)
		# print(float(speed.data))
		# converted_speed = float(speed.data)
		if abs(speed) < 0.01:
			channel.duty_cycle = 0
			return

		if speed > 0:
			channel.duty_cycle = int(self.speed_to_pwm(speed))

		print(int(self.speed_to_pwm(speed)))

def front_right_pwm_callback(speed):
	rospy.loginfo(rospy.get_caller_id() + "The velocities are %s",
					speed)
	pwm_driver.set_motors(float(speed.data), 
		Motor_Channel.front_right)

def front_left_pwm_callback(speed):
	rospy.loginfo(rospy.get_caller_id() + "The velocities are %s",
					speed)
	pwm_driver.set_motors(float(speed.data), 
		Motor_Channel.front_left)

def rear_right_pwm_callback(speed):
	rospy.loginfo(rospy.get_caller_id() + "The velocities are %s",
					speed)
	pwm_driver.set_motors(float(speed.data), 
		Motor_Channel.rear_right)

def rear_left_pwm_callback(speed):
	rospy.loginfo(rospy.get_caller_id() + "The velocities are %s",
					speed)
	pwm_driver.set_motors(float(speed.data), 
		Motor_Channel.rear_left)

if __name__ == '__main__':
	try:
		rospy.init_node('pwm')
		rospy.loginfo("Initialising pwm node")
		pwm_driver = PWM_Driver()
		print("Initialising pwm node")
		sub_motor_front_left = rospy.Subscriber('motor/front/left', Float32, front_left_pwm_callback)
		sub_motor_front_right = rospy.Subscriber('motor/front/right', Float32, front_right_pwm_callback)
		sub_motor_rear_left = rospy.Subscriber('motor/rear/left', Float32, rear_left_pwm_callback)
		sub_motor_rear_right = rospy.Subscriber('motor/rear/right', Float32, rear_right_pwm_callback)

		print("Initializing the instance!")
		rospy.loginfo("Initialised pwm node")
		rate = rospy.Rate(60)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

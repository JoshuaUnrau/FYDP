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
from std_msgs.msg import Float32
from std_msgs.msg import Int32

# in 1 is forward
# in 2 is backwards on motor driver

# actually no the motors are wired inconsistently
# AB pins around the motors are wired randomly

class Motor_Channel(IntEnum):
	front_right_forward = 0
	front_right_backward = 1
	rear_right_forward = 2
	rear_right_backward = 3
	
	linear_up = 4
	linear_down = 5

	servo = 6

	front_left_forward = 12
	front_left_backward = 13
	rear_left_forward = 14
	rear_left_backward = 15

class LimitSwitchState(IntEnum):
	Middle = 0
	Up = 1
	Down = 2

class PWM_Driver:
	def __init__(self):
		self.linear_actuator_state_pub = rospy.Publisher('current_lifter_state', Int32, queue_size=10)
		self.limitSwitchState = LimitSwitchState.Middle

		# Try to great a Digital input
		self.pin = digitalio.DigitalInOut(board.D4)
		print("Digital IO ok!")

		self.i2c = busio.I2C(board.SCL_1, board.SDA_1)
		print("I2C ok!")

		self.kit = ServoKit(channels=16, i2c=i2c)

		print("Connecting to PWM")
		self.pca = adafruit_pca9685.PCA9685(self.i2c)
		self.pca.frequency = 60


	#251 rpm @ 12V
	#Wheel diameter is 0.5m
	def speed_to_pwm(self, speed):
		max_speed = 251/60*0.5*math.pi
		speed_fraction = speed/max_speed
		if speed > max_speed:
			print("Trying to go faster than max speed")
		
		pwm_max = 65536 #pwm max value
		return speed_fraction*pwm_max

	#Speed is given in m/s
	def set_motors(self, speed, forward_channel_num, backward_channel_num):
		forward_channel = self.pca.channels[int(forward_channel_num)]
		backward_channel = self.pca.channels[int(backward_channel_num)]
		# print("HERE")
		# print(speed)
		# print(speed.data)
		# print(float(speed.data))
		# converted_speed = float(speed.data)
		if abs(speed) < 0.01:
			forward_channel.duty_cycle = 0
			backward_channel.duty_cycle = 0
			return
		
		if speed > 0:
			forward_channel.duty_cycle = int(self.speed_to_pwm(speed))
			backward_channel.duty_cycle = 0

		if speed < 0:
			forward_channel.duty_cycle = 0
			backward_channel.duty_cycle = int(self.speed_to_pwm(-speed))

		print(int(self.speed_to_pwm(speed)))
	
	def set_linear_actuator(self, direction):
		up_channel = self.pca.channels[int(Motor_Channel.linear_up)]
		down_channel = self.pca.channels[int(Motor_Channel.linear_down)]

		#Down
		if direction == 2:
			#At boundary
			if self.limitSwitchState == LimitSwitchState.Down:
				up_channel.duty_cycle = 0
				down_channel.duty_cycle = 0
			else:
				up_channel.duty_cycle = 0
				down_channel.duty_cycle = 65534

		#Up	
		if direction == 1:
			if self.limitSwitchState == LimitSwitchState.Up:
				up_channel.duty_cycle = 0
				down_channel.duty_cycle = 0
			else:
				up_channel.duty_cycle = 65534
				down_channel.duty_cycle = 0
		#Zero
		if direction == 0:
			up_channel.duty_cycle = 0
			down_channel.duty_cycle = 0

		self.linear_actuator_state_pub(self.limitSwitchState)

	def set_servo(self, angle):
		#This angle doesnt exactly agree with the provided angle
		#Perhaps due to the servo having a range that isnt 180
		#Or that the servo pwm range goes to ~2350
		kit.servo[Motor_Channel.servo] = angle

def front_right_pwm_callback(speed):
	rospy.loginfo(rospy.get_caller_id() + "The velocities are %s",
					speed)
	pwm_driver.set_motors(float(speed.data), 
		Motor_Channel.front_right_forward, 
		Motor_Channel.front_right_backward)

def front_left_pwm_callback(speed):
	rospy.loginfo(rospy.get_caller_id() + "The velocities are %s",
					speed)
	pwm_driver.set_motors(float(speed.data), 
		Motor_Channel.front_left_forward, 
		Motor_Channel.front_left_backward)

def rear_right_pwm_callback(speed):
	rospy.loginfo(rospy.get_caller_id() + "The velocities are %s",
					speed)
	pwm_driver.set_motors(float(speed.data), 
		Motor_Channel.rear_right_forward, 
		Motor_Channel.rear_right_backward)

def rear_left_pwm_callback(speed):
	rospy.loginfo(rospy.get_caller_id() + "The velocities are %s",
					speed)
	pwm_driver.set_motors(float(speed.data), 
		Motor_Channel.rear_left_forward, 
		Motor_Channel.rear_left_backward)

def linear_actuator_callback(direction):
	pwm_driver.set_linear_actuator(float(direction.data))
	
def servo_motor_callback(angle):
	pwm_driver.set_servo(angle.data)

def limit_switch_callback(state):
	if state == 2:
		pwm_driver.limitSwitchState = LimitSwitchState.Down
	if state == 1:
		pwm_driver.limitSwitchState = LimitSwitchState.Up
	if state == 0:
		pwm_driver.limitSwitchState = LimitSwitchState.Middle

pwm_driver = PWM_Driver()
if __name__ == '__main__':
	try:
		rospy.init_node('pwm')
		rospy.loginfo("Initialising pwm node")
		print("Initialising pwm node")
		sub_motor_front_left = rospy.Subscriber('motor/front/left', Float32, front_left_pwm_callback)
		sub_motor_front_right = rospy.Subscriber('motor/front/right', Float32, front_right_pwm_callback)
		sub_motor_rear_left = rospy.Subscriber('motor/rear/left', Float32, rear_left_pwm_callback)
		sub_motor_rear_right = rospy.Subscriber('motor/rear/right', Float32, rear_right_pwm_callback)
		sub_servo = rospy.Subscriber('servo', Int32, servo_motor_callback)
		sub_linear_actuator = rospy.Subscriber('desired_lifter_state', Int32, linear_actuator_callback)
		limit_switch = rospy.Subscriber('limit_switch', Int32, limit_switch_callback)

		print("Initializing the instance!")
		rospy.loginfo("Initialised pwm node")
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

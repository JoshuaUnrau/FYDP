#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from enum import IntEnum
import serial 
import time

#arduino = serial.Serial('/dev/ttyUSB2', 57600, timeout=1)
#Check usb devices:
#sudo udevadm monitor -u
#lsusb -t
#ls -l /dev/ttyUSB
#fuser -k /dev/ttyUSB9
#10% overestimation

def load_cell_pub():
    pub_limit = rospy.Publisher('limit_switch', Int32, queue_size=10)
    pub_neg_x = rospy.Publisher('loadcell_neg_x', Int32, queue_size=10)
    pub_pos_x = rospy.Publisher('loadcell_pos_x', Int32, queue_size=10)
    pub_y = rospy.Publisher('loadcell_y', Int32, queue_size=10)
    rospy.init_node('load_cell', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    print("Connecting to ardunio")
    arduino = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

    while not rospy.is_shutdown():
        try:
            data = arduino.read(40)
            string_val = ""
            D_found = False
            for char in data:
                if char == "D":
                    D_found = True
                    string_val = ""
                elif char == "F" and D_found:
                    break
                else:
                    string_val += char
            print(string_val)
            if data[0] != 's':
                try:
                    values = string_val.split(",")
                    limit_switch_up = int(values[0])
                    limit_switch_down = int(values[1])

                    limit_switch_value = 0
                    if (limit_switch_up == 0):
                        limit_switch_value = 1
                    if (limit_switch_down == 0):
                        limit_switch_value = 2
                    # if (limit_switch_down == 1 and limit_switch_up == 1):
                    #     limit_switch_value = 3
                    # if (limit_switch_down == 0 and limit_switch_up == 0):
                    #     limit_switch_value = 4

                    load_cell_1 = int(values[2])
                    load_cell_2 = int(values[3])
                    load_cell_3 = int(values[4])

                    print(load_cell_3)

                    pub_limit.publish(limit_switch_value)
                    pub_neg_x.publish(load_cell_1)
                    pub_pos_x.publish(load_cell_2)
                    pub_y.publish(load_cell_3)
                    rate.sleep()
                except:
                    print("Values parse error")
            else:
                print(data)
            arduino.reset_input_buffer()
        except Exception as e:
            time.sleep(1)
            print("Exception with ardunio connection: " + str(e))

if __name__ == '__main__':
    try:
        load_cell_pub()
    except rospy.ROSInterruptException:
        pass
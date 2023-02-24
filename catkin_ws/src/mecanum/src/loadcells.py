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
    pub_neg_x = rospy.Publisher('loadcell_neg_x', Int32, queue_size=10)
    pub_pos_x = rospy.Publisher('loadcell_pos_x', Int32, queue_size=10)
    pub_y = rospy.Publisher('loadcell_y', Int32, queue_size=10)
    rospy.init_node('load_cell', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    print("Connecting to ardunio")
    arduino = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)

    while not rospy.is_shutdown():
        try:
            data = arduino.readline()
            # print(int(data)/1.1)
            # Prepend text strings with s
            if data[0] != 's':
                try:
                    values = data.split(",")
                    load_cell_1 = int(values[0])
                    load_cell_2 = int(values[1])
                    load_cell_3 = int(values[2])
                    # print(str(load_cell_1) + ", " + str(load_cell_2) + ", " + str(load_cell_3))
                    pub_neg_x.publish(load_cell_1)
                    pub_pos_x.publish(load_cell_2)
                    pub_y.publish(load_cell_3)
                    rate.sleep()
                except:
                    print("Values parse error")
            else:
                print(data)
        except Exception as e:
            time.sleep(1)
            print("Exception with ardunio connection: " + str(e))

if __name__ == '__main__':
    try:
        load_cell_pub()
    except rospy.ROSInterruptException:
        pass
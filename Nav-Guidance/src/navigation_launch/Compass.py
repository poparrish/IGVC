#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from time import sleep
import serial
import math
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import sched, time
from std_msgs.msg import Int32

COMPASS_NODE = 'COMPASS_NODE'
ser = serial.Serial('/dev/ttyACM0', 115200)

HEADING_BIAS = 0

# Author: parker
# This ROS Node reads compass data from the arduino compass module21

# s = sched.scheduler(time.time, time.sleep)

# def print_data(sc):
#     heading = ser.readline()
#     s.enter(1,1, print_data,(sc,))

# s.enter(1,1,print_data,(s,))
# s.run()

def update_bias(data):
    global HEADING_BIAS
    HEADING_BIAS = data.data
    print(HEADING_BIAS)


def start():
    global HEADING_BIAS
    HEADING_BIAS = 0
    rospy.init_node(COMPASS_NODE)
    heading = rospy.Publisher('heading', Int32, queue_size=3)
    rospy.Subscriber('HEADING_BIAS', Int32, update_bias)
    r = rospy.Rate(10)
    newData = True
    while not rospy.is_shutdown():
        newHeading = ser.readline()
        #print(newHeading)
        try:
            int_heading = int(newHeading)
            print(HEADING_BIAS)
            correct_heading = (int_heading + HEADING_BIAS) % 360
            print("correct_heading: " + str(correct_heading))
            heading.publish(correct_heading)
        except: 
            pass
        r.sleep()


if __name__ == '__main__':
    start()
    
    






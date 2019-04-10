#!/usr/bin/env python
from sensor_msgs.msg import Joy
import math
import rospy
from std_msgs.msg import String
import serial
import os
import getpass
from geometry_msgs.msg import Vector3

#this just grabs data from the Arduino. Set frequency in start() arduino should run faster than
#ros, but its not required.

ser = serial.Serial('/dev/ttyUSB0',9600)

def getLatestData():#with dict lookup. perhaps consider using an assert:ser.inWaiting. Avoid duplicate data=bad map
    """
    Return dict
    :return:dict
    """

    while ser.inWaiting() > 0:
        toParse = ser.readline()
        parsed = toParse.split(",")
        #print "Length of Parsed: ",len(parsed)
        data = {
            'orientation': int(parsed[0]),
            'calibration_status': int(parsed[1]),#0 is not calibrated 3 is calibrated
            'is_alive': int(parsed[2])#0 if dead 1 if transmitting
        }
        return data

def start():
    """
    main method. where the ROS publishers and subscribers live.
    :return: nothing
    """
    pub = rospy.Publisher("orientation",Vector3,queue_size=1)
    rospy.init_node('compass_node')
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            orientation = getLatestData()
            ser.reset_input_buffer()
            vectorMsg = Vector3(x=orientation['orientation'],y=orientation['calibration_status'],z=orientation['is_alive'])
            print vectorMsg
            pub.publish(vectorMsg)
        except Exception as e:
            print 'failed to access compass %s' % e
            print e
        r.sleep()
    rospy.spin()

if __name__ == '__main__':
    start()

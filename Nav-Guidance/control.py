#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from time import sleep
import serial
import math

CONTROL_NODE = "CONTROL"
ser = serial.Serial('/dev/ttyACM0', 250000)

# Author: parker
# This ROS Node selects/converts inputs from the joy_node into wheel_speed and wheel_theta for the teensy

def callback(data): # Receives joystick messages subscribed to Joy topic
    WIDTH = 0.6223 
    LENGTH = 0.66675
    WHEELRADIUS = 0.0889  # no orange tread

    #xcoords for wheels
    w0x = WIDTH/2
    w1x = WIDTH/2
    w2x = WIDTH/2*-1
    w3x = WIDTH/2*-1
    #ycoords for wheels
    w0y = LENGTH/2
    w1y = LENGTH/2*-1
    w2y = LENGTH/2*-1
    w3y = LENGTH/2
    
    #THESE ARE JOYSTICK ONLY
    yAxis = data.axes[1] * 2
    xAxis = data.axes[0] * 2 * -1 
    
    #THESE ARE THE VALUES THAT GUIDANCE WILL PLUG INTO
    translation_theta = calcJoyBearing(xAxis,yAxis)#crabbing angle 
    theta_dot = data.axes[2] * -30#angle of rotation (not a vector any more) determines where on xaxis our external point of rotation is 30 degree turn max since that translates to a 45 degree inner wheel angle at benders chassis dimensions, this is sin(.5) of theta_dot
    bender_speed = data.axes[3] * 2.5#translational velocity at center of bender 

    #print str('bearing '+str(translation_theta))
    ser.write(str('W1'+calcWheel(bender_speed,translation_theta,w1x,w1y, theta_dot,LENGTH)+'\n'))

    ser.write(str('W0'+calcWheel(bender_speed,translation_theta,w0x,w0y, theta_dot,LENGTH)+'\n'))

    ser.write(str('W2'+calcWheel(bender_speed,translation_theta,w2x,w2y, theta_dot,LENGTH)+'\n'))

    ser.write(str('W3'+calcWheel(bender_speed,translation_theta,w3x,w3y, theta_dot,LENGTH)+'\n'))

def start():
    rospy.Subscriber(CONTROL_NODE, Joy, callback)#use this if joystick
    rospy.init_node(CONTROL_NODE)
    rospy.spin()

if __name__ == '__main__':
    start()
    
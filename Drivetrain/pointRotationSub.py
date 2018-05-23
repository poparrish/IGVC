#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from time import sleep
import serial
import math
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

ser = serial.Serial('/dev/ttyACM0', 250000)

# Author: parker
# This ROS Node selects/converts inputs from what will be the guidance node (right now it is subbed to our dummy node "floats to test pub/sub) and converts that into speeds and bearings for our wheels

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
    #yAxis = data.axes[1] * 2
    #xAxis = data.axes[0] * 2 * -1 
    
    #THESE ARE THE VALUES THAT GUIDANCE WILL PLUG INTO
    #translation_theta = calcJoyBearing(xAxis,yAxis)#crabbing angle 
    #theta_dot = data.axes[2] * -30#angle of rotation (not a vector any more) determines where on xaxis our external point of rotation is 30 degree turn max since that translates to a 45 degree inner wheel angle at benders chassis dimensions, this is sin(.5) of theta_dot
    #bender_speed = data.axes[3] * 2.5#translational velocity at center of bender 

    translation_theta = data.data[0]
    theta_dot = data.data[1]
    bender_speed = data.data[2]

    #print str('bearing '+str(translation_theta))
    ser.write(str('W1'+calcWheel(bender_speed,translation_theta,w1x,w1y, theta_dot,LENGTH)+'\n'))

    ser.write(str('W0'+calcWheel(bender_speed,translation_theta,w0x,w0y, theta_dot,LENGTH)+'\n'))

    ser.write(str('W2'+calcWheel(bender_speed,translation_theta,w2x,w2y, theta_dot,LENGTH)+'\n'))

    ser.write(str('W3'+calcWheel(bender_speed,translation_theta,w3x,w3y, theta_dot,LENGTH)+'\n'))

    
    
def calcWheel(bender_speed,translation_theta,wheelx,wheely,theta_dot,LENGTH): # Calculate wheel_theta and wheel_speed with direction
    #all calculations are in radians and meters. we convert to degrees and rpm at the end
    translation_theta_degrees = translation_theta
    translation_theta = translation_theta *0.0174533
    theta_dot_degrees = theta_dot
    theta_dot = theta_dot*0.0174533
    abs_trans_theta = math.fabs(theta_dot)+math.fabs(translation_theta)
    wheel_speed = 0
    wheel_theta = 0
    bender_radius = 0
    if(bender_speed < .05 and bender_speed > -.05):#bender_speed = 0 +-.05m/s DEADBAND
	if(theta_dot_degrees < 1 and theta_dot_degrees > -1):#theta_dot = 0degrees +-1degDEADBAND
	    wheel_theta = translation_theta
	else: #theta_dot != 0degrees

	    #if(abs_trans_theta > 0.785398):#we need to cap theta_dot at 45 degrees so that our point of rotation does not enter benders chassis.
		#if(theta_dot > 0):#case for positive theta_dot
		#    theta_dot = 0.785398-math.fabs(translation_theta)
		#else:#case for negative theta_dot
		#    theta_dot = -0.785398-math.fabs(translation_theta)

	    bender_radius = (LENGTH/2)/math.sin(theta_dot)#point on xaxis we are rotating around
	    wheel_theta = math.atan(wheely/(bender_radius - wheelx))+translation_theta#calculate the wheels angle to make a perpendicular with bender_radius. Then add any translationa_theta. Adding a translation_theta will rotate our point of rotation around bender so that we will have the option to pivot around our front or rear wheels if things get tight.
	    wheel_speed = 0 
    else: # bender_speed != 0
	if(theta_dot_degrees < 1 and theta_dot_degrees > -1):#theta_dot = 0degrees +-1degDEADBAND
	    wheel_theta = translation_theta
	    wheel_speed = bender_speed
	else: #theta_dot != 0degrees

	    #if(abs_trans_theta > 0.785398):#we need to cap theta_dot at 45 degrees so that our point of rotation does not enter benders chassis.
		#if(theta_dot > 0):#case for positive theta_dot
		#    theta_dot = 0.785398-math.fabs(translation_theta)
		#else:#case for negative theta_dot
		#    theta_dot = -0.785398-math.fabs(translation_theta)

	    bender_radius = (LENGTH/2)/math.tan(theta_dot)#point on xaxis we are rotating around distance from center of bender (changed sin to tan) needed the distance from the center not from the front
	    #we need to cap translation theta so that none of our wheels exceed 90 degrees
	    if(translation_theta_degrees > 20):
		translation_theta = 0.349066
	    if(translation_theta_degrees < -20):
		translation_theta = -0.349066

	    wheel_theta = math.atan(wheely/(bender_radius - wheelx))+translation_theta #calculate the wheels angle to make a perpendicular with bender_radius. Then add any translationa_theta. Adding a translation_theta will rotate our point of rotation around bender so that we will have the option to pivot around our front or rear wheels if things get tight.
	    
	    #here we use speed=distance/time to calculate the appropriate speed for each wheel. First we need to calculate the total distance the wheel will travel relative the center of the robot if bender drives in a full circle
	    wheelx = wheelx*-1
	    wheel_radius = math.sqrt(wheely*wheely+(bender_radius + wheelx)*(bender_radius + wheelx))
	    wheel_circumference = wheel_radius*2*3.14159#total distance center of robot will travel in meters
	    #we need to calculate the total time it should take the center point of bender to travel the wheel_circumference. This is the time it will take all the wheels to travel the circle if they are traveling at the right speeds
	    bender_radius = math.fabs(bender_radius)
	    bender_circumference = bender_radius*2*3.14159
	    travel_time = bender_circumference/bender_speed#travel time for all wheels
	    #NEW TEST LOGI
	    wheel_speed = wheel_circumference/travel_time#s=d/t
	    

    #convert from m/s to rpm & radians to degrees
    wheel_speed = wheel_speed/0.0085922#for no orange tread
    wheel_theta = wheel_theta*57.2958
    wheelString = str('B'+str(int(wheel_theta))+'S'+str(int(wheel_speed)))
    return wheelString

def start():
    rospy.Subscriber("floats", numpy_msg(Floats), callback)
    rospy.init_node('Joy2BenderBitches')
    rospy.spin()

if __name__ == '__main__':
    start()
    
    





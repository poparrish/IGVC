#!/usr/bin/env python
import rospy
from time import sleep
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy
#test publisher for guidance to pointRotationSub.py

pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=3)
rospy.init_node('talker', anonymous=True)
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    translation_theta = 0#this is crabbing angle
    theta_dot = 0#this is point rotation angle   
    bender_speed =0#this is speed at center of bender in m/s DO NOT EXCEED 2.2m/s. that is about 5mph
    a = numpy.array([translation_theta, theta_dot, bender_speed], dtype=numpy.float32)
    pub.publish(a)
    r.sleep()
    

#!/usr/bin/env python
import pickle

import rospy
from std_msgs.msg import String

import topics

if __name__ == '__main__':
    rospy.init_node('fake_gps')
    pub = rospy.Publisher(topics.GPS, String, queue_size=10)
    rate = rospy.Rate(10)

    msg = pickle.dumps({'lat': 0,
                        'lon': 0,
                        'heading': 10,
                        'satellites': 5,
                        'fixed': True})

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

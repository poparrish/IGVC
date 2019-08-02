#!/usr/bin/env python
import pickle

import rospy
from std_msgs.msg import String

import topics
from util import Vec2d

if __name__ == '__main__':
    rospy.init_node('fake_lidar')
    pub = rospy.Publisher(topics.LIDAR, String, queue_size=10)
    rate = rospy.Rate(10)

    msg = pickle.dumps([Vec2d(x, 1000) for x in xrange(-45, 45)])

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

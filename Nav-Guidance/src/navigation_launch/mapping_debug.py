#!/usr/bin/env python
import pickle

import cv2
import rospy
from roboviz import MapVisualizer
from std_msgs.msg import String
from sensor_msgs.msg import Image

from topics import MAP
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from cv_bridge import CvBridge, CvBridgeError

map_data = None

bridge = CvBridge()

def data_updated(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
        print(e)

    cv2.circle(cv_image, (50, 50), 10, 255)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('mapping_debug')

    rospy.Subscriber('/path', Image, data_updated)

    rospy.spin()
#!/usr/bin/env python
import pickle

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

import topics
from costmap_msg import CostmapData
from mapping import MAP_SIZE_METERS, MAP_SIZE_PIXELS
from util import rx_subscribe

COSTMAP_DILATION_M = 1.3  # closest distance to obstacles pathfinding is allowed to get


def build_costmap(map_data):
    # dilate obstacles
    dilation = int(float(COSTMAP_DILATION_M) / MAP_SIZE_METERS * MAP_SIZE_PIXELS)
    kernel = np.ones((dilation, dilation), np.uint8)
    img = map_data.map_bytes
    img = cv2.bitwise_not(img)
    ret, img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)
    img = cv2.dilate(img, kernel, iterations=1)
    img = cv2.bitwise_not(img)
    cv2.circle(img, (MAP_SIZE_PIXELS / 2, MAP_SIZE_PIXELS / 2), dilation / 2, (255,), -1)

    return CostmapData(transform=map_data.transform,
                       costmap_bytes=img,
                       map_bytes=map_data.map_bytes)


def start():
    rospy.init_node('costmap')

    pub = rospy.Publisher(topics.COSTMAP, String, queue_size=1)

    bridge = CvBridge()
    rviz = rospy.Publisher(topics.COSTMAP + "_rviz", Image, queue_size=1)

    def publish_costmap((lidar, camera)):
        costmap = build_costmap(lidar + camera)
        pub.publish(pickle.dumps(costmap))
        rviz.publish(bridge.cv2_to_imgmsg(costmap.costmap_bytes, 'mono8'))

    lidar_map = rx_subscribe(topics.MAP, String)
    camera_map = rx_subscribe(topics.LANE_MAP, String)
    lidar_map.combine_latest(camera_map, lambda l, c: (l, c)) \
        .throttle_first(150) \
        .subscribe(publish_costmap)

    rospy.spin()


if __name__ == '__main__':
    start()

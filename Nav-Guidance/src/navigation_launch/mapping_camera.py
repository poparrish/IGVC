#!/usr/bin/env python
import rospy

import topics
from mapping import publish_map, MapPublisher, MAXED, create_map
from util import rx_subscribe


def start_mapping():
    rospy.init_node('camera_mapping')

    camera_map = create_map(detection_angle_degrees=120, detection_margin=5)
    camera_data = rx_subscribe(topics.CAMERA).map(lambda msg: MAXED + msg.contours)
    publish_map(MapPublisher(camera_map, topics.LANE_MAP), camera_data)

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

#!/usr/bin/env python
import rospy

import topics
from mapping import publish_map, MapPublisher, create_map
from util import rx_subscribe


def start_mapping():
    rospy.init_node('lidar_mapping')

    lidar_map = create_map(detection_angle_degrees=270, detection_margin=5)
    lidar_data = rx_subscribe(topics.LIDAR)
    publish_map(MapPublisher(lidar_map, topics.MAP, tf_frame=topics.MAP_FRAME),
                scans=lidar_data)

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

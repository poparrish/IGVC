#!/usr/bin/env python
import rospy

import topics
from mapping import publish_map, MapPublisher, create_map, MAXED


def start_mapping():
    rospy.init_node('lidar_mapping')

    lidar_map = create_map(detection_angle_degrees=270, detection_margin=5)
    publish_map(MapPublisher(lidar_map, topics.MAP, tf_frame=topics.MAP_FRAME),
                topic=topics.LIDAR,
                process_msg=lambda msg: MAXED + msg)

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

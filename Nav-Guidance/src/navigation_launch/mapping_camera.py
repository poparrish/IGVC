#!/usr/bin/env python
import rospy

import topics
from mapping import publish_map, MapPublisher, MAXED, create_map


def start_mapping():
    rospy.init_node('camera_mapping')

    camera_map = create_map(detection_angle_degrees=120, detection_margin=5)
    publish_map(MapPublisher(camera_map, topics.LANE_MAP),
                topic=topics.CAMERA,
                process_msg=lambda msg: MAXED + msg.contours)

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

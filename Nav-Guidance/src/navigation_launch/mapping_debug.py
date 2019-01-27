#!/usr/bin/env python
import pickle

import rospy
from roboviz import MapVisualizer
from std_msgs.msg import String

from constants import MAP_TOPIC
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS

map_data = None


def data_updated(msg):
    global map_data
    map_data = pickle.loads(msg.data)


if __name__ == '__main__':
    rospy.init_node('mapping_debug')

    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')
    rospy.Subscriber(MAP_TOPIC, String, data_updated)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if map_data is None:
            continue

        x = map_data['x']
        y = map_data['y']
        theta = map_data['theta']
        map_bytes = map_data['map']

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(x / 1000., y / 1000., theta, map_bytes):
            exit(0)
        rate.sleep()

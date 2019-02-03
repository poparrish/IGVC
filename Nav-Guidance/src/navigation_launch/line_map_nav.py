#!/usr/bin/env python
import pickle

import math
import numpy as np
import rospy
from cameras import CAMERA_NODE
from util import rx_subscribe
from rx import Observable
from std_msgs.msg import String
from lidar import LIDAR_TOPIC
from lidar import start_lidar_noROS

from guidance import contours_to_vectors

LINE_MAP_HZ=5
LINE_MAP_NAV_NODE='LINE_MAP_NAV_NODE'



def process_state(camera, lidar):
    return {'camera': contours_to_vectors(camera.contours),
            'lidar': lidar}


def main():
    #start_lidar_noROS('/dev/lidar')
    rospy.init_node(LINE_MAP_NAV_NODE)
    pub = rospy.Publisher(LINE_MAP_NAV_NODE, String, queue_size=10)

    camera = rx_subscribe(CAMERA_NODE)
    lidar = rx_subscribe(LIDAR_TOPIC)
    #camera = BehaviorSubject(CameraMsg(contours=[]))
    #lidar = BehaviorSubject([])
    #gps = rx_subscribe(GPS_NODE)

    rospy.loginfo('Nav waiting for messages...')
    combined = Observable.combine_latest(camera, lidar, process_state)# \
        #.throttle_first(1000.0 / NAV_HZ)

    combined.take(1).subscribe(lambda x: rospy.loginfo('Nav starting...'))
    combined.tap(rospy.loginfo).subscribe(lambda x: pub.publish(pickle.dumps(x)))

    rospy.spin()


if __name__ == '__main__':
    main()


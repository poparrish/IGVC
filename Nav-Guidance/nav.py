#!/usr/bin/env python
import pickle

import rospy
from rx import Observable
from rx.subjects import BehaviorSubject
from std_msgs.msg import String

from camera_msg import CameraMsg
from cameras import CAMERA_NODE
from gps import GPS_NODE
from guidance import contours_to_vectors
from lidar import LIDAR_NODE
from util import rx_subscribe

NAV_HZ = 5
NAV_NODE = 'NAV'


def process_state(camera, lidar, gps):
    return {'camera': contours_to_vectors(camera.contours),
            'gps': gps,
            'lidar': lidar}


def main():
    rospy.init_node(NAV_NODE)
    pub = rospy.Publisher(NAV_NODE, String, queue_size=10)

    camera = rx_subscribe(CAMERA_NODE, String, pickle.loads)
    lidar = rx_subscribe(LIDAR_NODE, String, pickle.loads)
    # camera = BehaviorSubject(CameraMsg(contours=[]))
    # lidar = BehaviorSubject([])
    gps = rx_subscribe(GPS_NODE, String, pickle.loads)

    rospy.loginfo('Nav waiting for messages...')
    combined = Observable.combine_latest(camera, lidar, gps, process_state) \
        .throttle_first(1000.0 / NAV_HZ) \
        .tap(rospy.loginfo)

    combined.take(1).subscribe(lambda x: rospy.loginfo('Nav starting...'))
    combined.map(pickle.dumps).subscribe(pub.publish)

    rospy.spin()


if __name__ == '__main__':
    main()

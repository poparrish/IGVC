#!/usr/bin/env python
import pickle

import rospy
from rx import Observable
from std_msgs.msg import String

import topics
from guidance import contours_to_vectors
from util import rx_subscribe

NAV_HZ = 10
NAV_NODE = 'NAV'


def process_state(camera, lidar, gps):
    return {'camera': contours_to_vectors(camera.contours),
            'gps': gps,
            'lidar': lidar}


def main():
    rospy.init_node(NAV_NODE)
    pub = rospy.Publisher(NAV_NODE, String, queue_size=10)

    camera = rx_subscribe(topics.CAMERA)
    lidar = rx_subscribe(topics.LIDAR)
    # camera = BehaviorSubject(CameraMsg(contours=[]))
    # lidar = BehaviorSubject([])
    gps = rx_subscribe(topics.GPS)

    rospy.loginfo('Nav waiting for messages...')
    combined = Observable.combine_latest(camera, lidar, gps, process_state)# \
        #.throttle_first(1000.0 / NAV_HZ)

    combined.take(1).subscribe(lambda x: rospy.loginfo('Nav starting...'))
    combined.subscribe(lambda x: pub.publish(pickle.dumps(x)))

    rospy.spin()


if __name__ == '__main__':
    main()

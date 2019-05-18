#!/usr/bin/env python
import rospy
import tf
from tf.transformations import quaternion_from_euler

import topics


def start():
    rospy.init_node('fake_odom')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown():
        br.sendTransform(
            translation=(i, 0, 0),
            rotation=quaternion_from_euler(0, 0, 0),
            time=rospy.Time.now(),
            child=topics.ODOMETRY_FRAME,
            parent=topics.WORLD_FRAME
        )
        i += .03
        # br.sendTransform(
        #     translation=(1.5 + i * 2, 1, 0),
        #     rotation=quaternion_from_euler(0, 0, 1),
        #     time=rospy.Time.now(),
        #     child=topics.MAP_FRAME,
        #     parent=topics.WORLD_FRAME
        # )
        rate.sleep()


if __name__ == '__main__':
    start()

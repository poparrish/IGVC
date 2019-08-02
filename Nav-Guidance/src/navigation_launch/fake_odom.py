#!/usr/bin/env python
import rospy
import tf
from tf.transformations import quaternion_from_euler

import topics


def start():
    rospy.init_node('fake_odom')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    x = 0
    heading = 0
    while not rospy.is_shutdown():
        br.sendTransform(
            translation=(0.4, 0.45 + x, 0),
            rotation=quaternion_from_euler(0, 0, heading),
            time=rospy.Time.now(),
            child=topics.ODOMETRY_FRAME,
            parent=topics.WORLD_FRAME
        )
        x += .01
        # heading += 0.01
        rate.sleep()


if __name__ == '__main__':
    start()

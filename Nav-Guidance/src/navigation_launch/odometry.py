#!/usr/bin/env python
import rospy
import tf

ODOMETRY_NODE = 'ODOMETRY'

if __name__ == '__main__':
    rospy.init_node(ODOMETRY_NODE)
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         'odom',
                         'base_link')

#!/usr/bin/env python
import pickle
import time

import rospy
from rplidar import RPLidar, MAX_MOTOR_PWM, RPLidarException
from std_msgs.msg import String

import topics
from util import Vec2d

MIN_DIST_MM = 500
MAX_DIST_MM = 10000
LIDAR_OFFSET_MM = 450  # offset from center


def create_vector((quality, angle, dist)):
    angle = -angle + 360  # invert lidar
    return Vec2d(angle, dist)


def convert_scan_to_vectors(scan):
    return [v for v in map(create_vector, scan) if v is not None]


def start_lidar(device):
    rospy.init_node('lidar')
    pub = rospy.Publisher(topics.LIDAR, String, queue_size=1)

    lidar = RPLidar(device)
    lidar.stop_motor()
    time.sleep(1)

    lidar.set_pwm(MAX_MOTOR_PWM)
    lidar._serial_port.setDTR(False)
    lidar.set_pwm(MAX_MOTOR_PWM)

    # lidar.set_pwm(MAX_MOTOR_PWM)  # set to full speed

    while not rospy.is_shutdown():
        for i, scan in enumerate(lidar.iter_scans()):
            scan = convert_scan_to_vectors(scan)
            pub.publish(pickle.dumps(scan))


if __name__ == '__main__':
    # lidar randomly fails to start with "incorrect starting bytes descriptor" error
    # retry a few times
    error = None
    for x in xrange(3):
        try:
            error = None
            start_lidar('/dev/lidar')
        except RPLidarException as e:
            error = e
            rospy.logwarn('Failed to start lidar, trying again: %s', e)

    if error is not None:
        raise error

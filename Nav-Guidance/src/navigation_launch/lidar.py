#!/usr/bin/env python
import pickle
import time

import rospy
from rplidar import RPLidar, MAX_MOTOR_PWM, RPLidarException
from std_msgs.msg import String

from constants import LIDAR_TOPIC
from util import Vec2d

MIN_DIST_MM = 500
MAX_DIST_MM = 10000
LIDAR_OFFSET_MM = 450  # offset from center

ANGLE_IGNORE_WINDOW = 45  # ignore readings +/- this angle behind Bender


def create_vector((quality, angle, dist)):
    # valid_angle = ANGLE_IGNORE_WINDOW < angle < 360 - ANGLE_IGNORE_WINDOW
    # valid_distance = MIN_DIST_MM < dist

    v = Vec2d(angle, dist)
    # v = v.from_point(v.x - LIDAR_OFFSET_MM, v.y)

    return v


def convert_scan_to_vectors(scan):
    return [v for v in map(create_vector, scan) if v is not None]


def start_lidar(device):
    rospy.init_node('lidar')
    pub = rospy.Publisher(LIDAR_TOPIC, String, queue_size=1)

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

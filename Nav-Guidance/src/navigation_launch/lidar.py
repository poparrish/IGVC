#!/usr/bin/env python
import pickle

import rospy
from rplidar import RPLidar, MAX_MOTOR_PWM
from std_msgs.msg import String

from util import clamp360, Vec2d

LIDAR_NODE = 'LIDAR'

MIN_DIST_MM = 100
MAX_DIST_MM = 2000

ANGLE_IGNORE_START = 120
ANGLE_IGNORE_END = 240

LIDAR_OFFSET_ANGLE = 180  # lidar is backward


def create_vector((quality, angle, dist)):
    angle = clamp360(angle + LIDAR_OFFSET_ANGLE)
    valid_angle = not ANGLE_IGNORE_START <= angle <= ANGLE_IGNORE_END
    valid_distance = MIN_DIST_MM < dist < MAX_DIST_MM
    if valid_angle and valid_distance:
        return Vec2d(angle, dist)


def convert_scan_to_vectors(scan):
    return [v for v in map(create_vector, scan) if v is not None]


def start_lidar(device):
    rospy.init_node(LIDAR_NODE)
    pub = rospy.Publisher(LIDAR_NODE, String, queue_size=1)

    lidar = RPLidar(device)
    lidar.set_pwm(MAX_MOTOR_PWM)  # set to full speed

    while not rospy.is_shutdown():
        for i, scan in enumerate(lidar.iter_scans()):
            scan = convert_scan_to_vectors(scan)
            pub.publish(pickle.dumps(scan))


if __name__ == '__main__':
    start_lidar('/dev/lidar')

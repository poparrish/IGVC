#!/usr/bin/env python
import math
import pickle

import rospy
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from std_msgs.msg import String

from constants import MAP_TOPIC, LIDAR_TOPIC, ODOMETRY_TOPIC
from lidar import ANGLE_IGNORE_WINDOW
from util import rx_subscribe

MAP_SIZE_PIXELS = 500
MAP_SIZE_METERS = 10

MIN_SAMPLES = 50


class RPLidarA1(Laser):
    def __init__(self, detection_margin=0, offset_millimeters=0):
        Laser.__init__(self, 360, 5.5, 360 - ANGLE_IGNORE_WINDOW * 2, 12000, detection_margin, offset_millimeters)


def update_map(slam, map_bytes, (previous, current)):
    _, previous_position, previous_time = previous
    scan, position, time = current

    # Extract distances and angles from lidar
    distances = [r.mag for r in scan]
    angles = [r.angle for r in scan]

    # Calculate pose_change
    # dxy_mm, dtheta_degrees, dt_seconds
    x = (position['x'] - previous_position['x']) * 1000.0
    y = (position['y'] - previous_position['y']) * 1000.0
    pose_change = (math.sqrt(x ** 2 + y ** 2),
                   0,  # TODO get from compass
                   time - previous_time)

    rospy.loginfo('Updating map, delta: %s', pose_change)

    slam.update(scans_mm=distances,
                scan_angles_degrees=angles,
                pose_change=pose_change)

    slam.getmap(map_bytes)
    x, y, theta = slam.getpos()

    return {'x': x, 'y': y, 'theta': theta, 'map': map_bytes}


def combine_readings(lidar, odometry):
    return lidar, odometry, rospy.get_rostime().to_sec()


def keep_scan(scan):
    return len(scan) > MIN_SAMPLES


def start_mapping():
    rospy.init_node('mapping')
    pub = rospy.Publisher(MAP_TOPIC, String, queue_size=1)

    # subscribe to nodes
    lidar = rx_subscribe(LIDAR_TOPIC).filter(keep_scan)
    odometry = rx_subscribe(ODOMETRY_TOPIC).start_with({'x': 0, 'y': 0})

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(
        laser=RPLidarA1(),
        map_size_pixels=MAP_SIZE_PIXELS,
        map_size_meters=MAP_SIZE_METERS,
        hole_width_mm=200,
        sigma_theta_degrees=0  # disable directional prediction; we have a compass
    )
    map_bytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    lidar.with_latest_from(odometry, combine_readings) \
        .pairwise() \
        .map(lambda data: update_map(slam, map_bytes, data)) \
        .subscribe(lambda msg: pub.publish(pickle.dumps(msg)))

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

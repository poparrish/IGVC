#!/usr/bin/env python
import math
import pickle
import traceback
from time import sleep

import cv2
import numpy as np
import rospy
import sensor_msgs
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rx import Observable
from rx.linq.observable.withlatestfrom import with_latest_from
from sensor_msgs.msg import Image
from std_msgs.msg import String

from constants import MAP_TOPIC, LIDAR_TOPIC, ODOMETRY_TOPIC, CAMERA_TOPIC
from guidance import contours_to_vectors
from lidar import ANGLE_IGNORE_WINDOW
from util import rx_subscribe, Vec2d

MAP_SIZE_PIXELS = 100
MAP_SIZE_METERS = 5

MIN_SAMPLES = 50
MAX_DIST_MM = 10000

SAFE_DIST_PIXELS = int(0.5 * MAP_SIZE_PIXELS / MAP_SIZE_METERS)

UNKNOWN = 127  # unmapped/unknown value set in map


class RPLidarA1(Laser):
    def __init__(self):
        Laser.__init__(self, scan_size=360, scan_rate_hz=5.5,
                       detection_angle_degrees=360 - ANGLE_IGNORE_WINDOW * 2,
                       distance_no_detection_mm=MAX_DIST_MM,
                       detection_margin=0,
                       offset_mm=0)


def in_range(scan):
    return ANGLE_IGNORE_WINDOW < scan.angle < 360 - ANGLE_IGNORE_WINDOW


def skew(angle):
    return (angle - ANGLE_IGNORE_WINDOW) * 180 / (180 - ANGLE_IGNORE_WINDOW)


def update_map(slam, map_bytes, (previous, current)):
    _, previous_position, previous_time = previous
    scan, position, time = current

    # Extract distances and angles from lidar
    scan = [s for s in scan if in_range(s)]
    distances = [s.mag for s in scan]
    angles = [skew(s.angle) for s in scan]

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


def convert_camera_contours(data):
    contours = contours_to_vectors(data.contours)
    return [p for contour in contours for p in contour]


def combine_points(lidar, camera):
    vecs = lidar + camera
    nearest = {}

    # make sure to provide a reading for all angles, otherwise Breezy
    # will try to fill in the gaps
    for x in xrange(0, 360):
        nearest[x] = Vec2d(x, MAX_DIST_MM)

    for v in vecs:
        angle = int(v.angle)

        if angle in nearest:
            old = nearest[angle]
            if old.mag > v.mag:
                nearest[angle] = v
        else:
            nearest[angle] = v

    return nearest.values()


def pixel_to_grid(x):
    return (255 - x) / 255.0 * 127.0 if x != UNKNOWN else -1


def pixel_to_byte(x):
    return -x * 255.0 / 127.0 + 255 if x != -1 else UNKNOWN


def publish(map_pub, msg):
    data = [pixel_to_grid(x) for x in msg['map']]
    map_pub.publish(OccupancyGrid(
        info=MapMetaData(resolution=float(MAP_SIZE_METERS) / MAP_SIZE_PIXELS,
                         width=MAP_SIZE_PIXELS,
                         height=MAP_SIZE_PIXELS,
                         origin=Pose(position=Point(x=MAP_SIZE_METERS / -2.0, y=MAP_SIZE_METERS / -2.0))),
        data=data))


def start_mapping():
    rospy.init_node('mapping')
    map_pub = rospy.Publisher(MAP_TOPIC, OccupancyGrid, queue_size=1)

    # subscribe to nodes
    lidar = rx_subscribe(LIDAR_TOPIC).filter(keep_scan).start_with([])
    camera = rx_subscribe(CAMERA_TOPIC).map(convert_camera_contours).start_with([])
    points = Observable.combine_latest(lidar, camera, combine_points)

    odometry = rx_subscribe(ODOMETRY_TOPIC).start_with({'x': 0, 'y': 0})

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(
        laser=RPLidarA1(),
        map_size_pixels=MAP_SIZE_PIXELS,
        map_size_meters=MAP_SIZE_METERS,
        hole_width_mm=500,
        sigma_theta_degrees=0,  # disable directional prediction; we have a compass
        sigma_xy_mm=0
    )
    map_bytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    points.with_latest_from(odometry, combine_readings) \
        .pairwise() \
        .map(lambda data: update_map(slam, map_bytes, data)) \
        .subscribe(on_next=lambda msg: publish(map_pub, msg),
                   on_error=lambda e: rospy.logerr(traceback.format_exc(e)))

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

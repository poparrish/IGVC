#!/usr/bin/env python
import math
import traceback

import rospy
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

import topics
from guidance import contours_to_vectors
from lidar import ANGLE_IGNORE_WINDOW
from util import rx_subscribe, Vec2d
import cv2
import numpy as np
import cameras

MAP_SIZE_PIXELS = 101
MAP_SIZE_METERS = 5

MIN_SAMPLES = 50
MAX_DIST_MM = 10000

UNKNOWN = 127  # unmapped/unknown value set in map


def fill_scan(scan):
    nearest = {}

    # make sure to provide a reading for all angles, otherwise Breezy
    # will try to fill in the gaps
    for x in xrange(0, 360):
        nearest[x] = Vec2d(x, MAX_DIST_MM)

    for v in scan:
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


class Map:
    def __init__(self, map_pub, pos_pub):
        # Create an RMHC SLAM object with a laser model and optional robot model
        self.slam = RMHC_SLAM(
            laser=Laser(scan_size=360, scan_rate_hz=5.5,
                        detection_angle_degrees=360 - ANGLE_IGNORE_WINDOW * 2,
                        distance_no_detection_mm=MAX_DIST_MM,
                        detection_margin=0,
                        offset_mm=0),
            map_size_pixels=MAP_SIZE_PIXELS,
            map_size_meters=MAP_SIZE_METERS,
            hole_width_mm=500,
            sigma_theta_degrees=0,  # disable directional prediction; we have a compass
            sigma_xy_mm=0
        )
        self.map_bytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.map_pub = rospy.Publisher(map_pub, OccupancyGrid, queue_size=1)
        self.pos_pub = rospy.Publisher(pos_pub, PoseStamped, queue_size=1)

    def update(self, (scan, dxy_mm, dtheta_degrees, dt_seconds)):
        # TODO: Update position w/o updating map if scans haven't changed
        if scan is None:
            return

        # Extract distances and angles from vectors
        scan = fill_scan([v for v in scan if self.in_range(v)])
        distances = [v.mag for v in scan]
        angles = [self.skew(v.angle) for v in scan]

        rospy.loginfo('Updating map, delta: %.2fmm, %.2f deg, %.2f sec', dxy_mm, dtheta_degrees,dt_seconds)

        self.slam.update(scans_mm=distances,
                         scan_angles_degrees=angles,
                         pose_change=(dxy_mm, dtheta_degrees, dt_seconds))
        self.slam.getmap(self.map_bytes)

    # TODO: Scan window should be configurable per-map
    def in_range(self, scan):
        return ANGLE_IGNORE_WINDOW < scan.angle < 360 - ANGLE_IGNORE_WINDOW

    def skew(self, angle):
        return (angle - ANGLE_IGNORE_WINDOW) * 180 / (180 - ANGLE_IGNORE_WINDOW)

    def get_bytes(self):
        return self.map_bytes

    def get_position(self):
        return self.slam.getpos()

    def publish_lanes(self):
        offset = MAP_SIZE_METERS / -2.0

        data = [pixel_to_grid(x) for x in self.map_bytes]

        a = np.array(data)
        npdata = np.reshape(a, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

        im = np.array(npdata, dtype=np.uint8)
        rows, cols = im.shape
        M = cv2.getRotationMatrix2D((cols / 2, rows / 2), 90, 1)
        rot_im = cv2.warpAffine(im, M, (cols, rows))
        inv_im = cv2.bitwise_not(rot_im)
        thresh_im = cv2.inRange(inv_im, 60, 200)
        radius = 1
        ksize = int(6 * round(radius) + 1)
        gauss_im = cv2.GaussianBlur(thresh_im, (ksize, ksize), round(radius))
        contours = cameras.find_contours(input=gauss_im, external_only=False)
        display_contours = np.ones_like(im)
        cv2.drawContours(display_contours, contours, -1, (255, 255, 255), thickness=1)
        contoursMinArea = 150
        contoursMinPerimeter = 1
        contoursMinWidth = 0
        contoursMaxWidth = 1000000
        contoursMinHeight = 0
        contoursMaxHeight = 1000000
        contoursSolidity = [21, 100]
        contoursSolidityMin = 21
        contoursSolidityMax = 100
        contoursMaxVertices = 1000000
        contoursMinVertices = 0
        contoursMinRatio = 0
        contoursMaxRatio = 10000
        filtered_contours = cameras.filter_contours(input_contours=contours, min_area=contoursMinArea,
                                                    min_perimeter=contoursMinPerimeter,
                                                    min_width=contoursMinWidth, max_width=contoursMaxWidth,
                                                    min_height=contoursMinHeight,
                                                    max_height=contoursMaxHeight,
                                                    solidity=[contoursSolidityMin, contoursSolidityMax],
                                                    max_vertex_count=contoursMaxVertices,
                                                    min_vertex_count=contoursMinVertices, min_ratio=contoursMinRatio,
                                                    max_ratio=contoursMaxRatio)
        display_filtered_contours = np.ones_like(im)
        cv2.drawContours(display_filtered_contours, filtered_contours, -1, (255, 255, 255), thickness=-1)
        cv2.imshow('contour', display_filtered_contours)

        cartesian_contours = cameras.convert_to_cartesian(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, contours)
        vec2d_contours = contours_to_vectors(cartesian_contours)
        lane_angle = cameras.closest_contour_slope(vec2d_contours)

        print "ANGLE: ", lane_angle

        cv2.imshow("raw_thresh", thresh_im)
        cv2.waitKey(3)

        self.map_pub.publish(OccupancyGrid(
            info=MapMetaData(resolution=float(MAP_SIZE_METERS) / MAP_SIZE_PIXELS,
                             width=MAP_SIZE_PIXELS,
                             height=MAP_SIZE_PIXELS,
                             origin=Pose(position=Point(x=offset, y=offset))),
            data=data))

        # TODO: Translate map once +/- threshold from center
        x, y, theta = self.get_position()

        self.pos_pub.publish(PoseStamped(header=Header(frame_id='map'),
                                         pose=Pose(position=Point(x=x / 1000 + offset, y=y / 1000 + offset),
                                                   orientation=Quaternion())))

    def publish(self):
        offset = MAP_SIZE_METERS / -2.0

        data = [pixel_to_grid(x) for x in self.map_bytes]

        self.map_pub.publish(OccupancyGrid(
            info=MapMetaData(resolution=float(MAP_SIZE_METERS) / MAP_SIZE_PIXELS,
                             width=MAP_SIZE_PIXELS,
                             height=MAP_SIZE_PIXELS,
                             origin=Pose(position=Point(x=offset, y=offset))),
            data=data))

        # TODO: Translate map once +/- threshold from center
        x, y, theta = self.get_position()

        self.pos_pub.publish(PoseStamped(header=Header(frame_id='map'),
                                         pose=Pose(position=Point(x=x / 1000 + offset, y=y / 1000 + offset),
                                                   orientation=Quaternion())))


def diff_state(state):
    prev_scan, previous_position, previous_time = state[0]
    scan, position, time = state[1]

    # Calculate pose_change
    dx_mm = (position['x'] - previous_position['x']) * 1000.0
    dy_mm = (position['y'] - previous_position['y']) * 1000.0
    dxy_mm = (math.sqrt(dx_mm ** 2 + dy_mm ** 2))
    dtheta_degrees = 0  # TODO: IMU
    dt_seconds = time - previous_time

    # if we didn't get new readings, only update our position
    # if scan == prev_scan:
    #     scan = None

    return scan, dxy_mm, dtheta_degrees, dt_seconds


def start_mapping():
    rospy.init_node('mapping')

    combined_map = Map(topics.MAP, topics.MAP_POSE)
    camera_map = Map(topics.CAMERA_MAP, topics.CAMERA_MAP_POSE)

    lidar = rx_subscribe(topics.LIDAR) \
        .filter(lambda scan: len(scan) > MIN_SAMPLES) \
        .start_with([])

    camera = rx_subscribe(topics.CAMERA) \
        .map(lambda msg: [v for contour in (contours_to_vectors(msg.contours)) for v in contour]) \
        .start_with([])

    no_barrels_camera = rx_subscribe(topics.NO_BARREL_CAMERA) \
        .map(lambda msg: [v for v in msg.contours]) \
        .start_with([])

    odometry = rx_subscribe(topics.ODOMETRY) \
        .start_with({'x': 0, 'y': 0})

    def publish_map(map, scans):
        return scans.with_latest_from(odometry, lambda v, o: (v, o, rospy.get_rostime().to_sec())) \
            .pairwise() \
            .map(diff_state) \
            .do_action(map.update) \
            .subscribe(on_next=lambda _: map.publish(),
                       on_error=lambda e: rospy.logerr(traceback.format_exc(e)))

    def publish_lane_map(map, scans):
        return scans.with_latest_from(odometry, lambda v, o: (v, o, rospy.get_rostime().to_sec())) \
            .pairwise() \
            .map(diff_state) \
            .do_action(map.update) \
            .subscribe(on_next=lambda _: map.publish_lanes(),
                       on_error=lambda e: rospy.logerr(traceback.format_exc(e)))

    publish_map(combined_map, lidar.with_latest_from(camera, lambda l, c: l + c))

    publish_lane_map(camera_map, no_barrels_camera)

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

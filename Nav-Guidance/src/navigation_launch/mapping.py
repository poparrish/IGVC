#!/usr/bin/env python
import math
import traceback

import cv2
import numpy as np
import rospy
import tf
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from geometry_msgs.msg import Pose, Point, TransformStamped, Transform, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

import cameras
import topics
from guidance import contours_to_vectors
from lidar import ANGLE_IGNORE_WINDOW
from util import rx_subscribe, Vec2d
from util.rosutil import extract_tf

MAP_SIZE_PIXELS = 101
MAP_SIZE_METERS = 5

MIN_SAMPLES = 50
MAX_DIST_MM = 10000
MAX_TRAVEL_M = 1 # TODO: Name

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

    return scan

def pixel_to_grid(x):
    return (255 - x) / 255.0 * 127.0 if x != UNKNOWN else -1


def pixel_to_byte(x):
    return -x * 255.0 / 127.0 + 255 if x != -1 else UNKNOWN


def occupancy_grid_to_np(grid):
    """Converts an OccupancyGrid to a 2-dimensional numpy array"""
    img = np.array([pixel_to_byte(x) for x in grid.data], dtype=np.uint8)
    return np.reshape(img, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), order='F')


def create_slam():
    return RMHC_SLAM(
        laser=Laser(scan_size=360, scan_rate_hz=5.5,
                    detection_angle_degrees=360 - ANGLE_IGNORE_WINDOW * 2,
                    distance_no_detection_mm=MAX_DIST_MM,
                    detection_margin=0,
                    offset_mm=0),
        map_size_pixels=MAP_SIZE_PIXELS,
        map_size_meters=MAP_SIZE_METERS,
        hole_width_mm=500,
        sigma_theta_degrees=0,  # disable directional prediction; we have a compass
        sigma_xy_mm=0)


def mm_to_pixels(mm):
    m = MAP_SIZE_METERS / 2.0 - mm / 1000.0
    return int(m / MAP_SIZE_METERS * MAP_SIZE_PIXELS)


class Map:
    def __init__(self, map_pub, tf_frame=None):
        # Create an RMHC SLAM object with a laser model and optional robot model
        self.slam = create_slam()
        self.map_bytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.map_pub = rospy.Publisher(map_pub, OccupancyGrid, queue_size=1)
        self.tf_frame = tf_frame
        if tf_frame is not None:
            self.br = tf.TransformBroadcaster()
        self.transform = TransformStamped(transform=Transform(rotation=Quaternion(0, 0, 0, 1)))

    def zero(self):
        """
        Zeros the map by translating map_bytes by the distance/theta travelled so far.

        https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html
        """
        img = np.reshape(self.map_bytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
        rows, cols = img.shape

        position = self.slam.position

        border_mode = cv2.BORDER_CONSTANT
        border_value = (UNKNOWN)

        # rotate
        matrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), position.theta_degrees, 1)
        img = cv2.warpAffine(img, matrix, (cols, rows), borderMode=border_mode, borderValue=border_value)

        # translate
        matrix = np.float32([[1, 0, mm_to_pixels(position.x_mm)],
                             [0, 1, mm_to_pixels(position.y_mm)]])
        img = cv2.warpAffine(img, matrix, (cols, rows), borderMode=border_mode, borderValue=border_value)

        # reset slam
        self.map_bytes = bytearray(np.reshape(img, (MAP_SIZE_PIXELS ** 2)))
        self.slam = create_slam()

    def update(self, (scan, dxy_mm, dtheta_degrees, dt_seconds, new_transform)):
        # Extract distances and angles from vectors
        scan = fill_scan([v for v in scan if self.in_range(v)])
        distances = [v.mag for v in scan]
        angles = [self.skew(v.angle) for v in scan]

        rospy.loginfo('Updating map, delta: %.2fmm, %.2f deg, %.2f sec', dxy_mm, dtheta_degrees, dt_seconds)

        self.slam.update(scans_mm=distances,
                         scan_angles_degrees=angles,
                         pose_change=(dxy_mm, dtheta_degrees, dt_seconds))
        self.slam.getmap(self.map_bytes)

        if max(abs(self.transform.transform.translation.x - new_transform.transform.translation.x),
               abs(self.transform.transform.translation.y - new_transform.transform.translation.y)) > MAX_TRAVEL_M:
            self.zero()
            self.transform = new_transform

    # TODO: Scan window should be configurable per-map
    def in_range(self, scan):
        return ANGLE_IGNORE_WINDOW < scan.angle < 360 - ANGLE_IGNORE_WINDOW

    def skew(self, angle):
        return (angle - ANGLE_IGNORE_WINDOW) * 180 / (180 - ANGLE_IGNORE_WINDOW)

    def get_bytes(self):
        return self.map_bytes

    def get_position(self):
        transform = self.transform.transform
        x, y, theta = self.slam.getpos()
        return x / 1000.0 - MAP_SIZE_METERS / 2.0 + transform.x, \
               y / 1000.0 - MAP_SIZE_METERS / 2.0 + transform.y, \
               theta

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
        self.publish_pose()

    def publish_pose(self):
        if self.br is not None:
            self.br.sendTransformMessage(TransformStamped(
                header=Header(frame_id=topics.WORLD_FRAME,
                              stamp=rospy.Time.now()),
                child_frame_id=topics.MAP_FRAME,
                transform=self.transform.transform))


def diff_state(state):
    prev_scan, previous_position, previous_time = state[0]
    scan, position, time = state[1]

    # Calculate pose_change
    dx_mm = (position.transform.translation.x - previous_position.transform.translation.x) * 1000.0
    dy_mm = (position.transform.translation.y - previous_position.transform.translation.y) * 1000.0
    dxy_mm = (math.sqrt(dx_mm ** 2 + dy_mm ** 2))
    dtheta_degrees = math.degrees(position.transform.rotation.z - previous_position.transform.rotation.z)
    dt_seconds = time - previous_time

    # if we didn't get new readings, only update our position
    # if scan == prev_scan:
    #     scan = None

    return scan, dxy_mm, dtheta_degrees, dt_seconds, position


def start_mapping():
    rospy.init_node('mapping')

    odometry = rx_subscribe('/tf', TFMessage, parse=None) \
        .let(extract_tf(topics.ODOMETRY_FRAME))

    combined_map = Map(topics.MAP, topics.MAP_FRAME)
    camera_map = Map(topics.LINE_MAP, None)

    lidar = rx_subscribe(topics.LIDAR) \
        .filter(lambda scan: len(scan) > MIN_SAMPLES) \
        .start_with([])

    camera = rx_subscribe(topics.CAMERA) \
        .map(lambda msg: [v for contour in (contours_to_vectors(msg.contours)) for v in contour]) \
        .start_with([])

    no_barrels_camera = rx_subscribe(topics.NO_BARREL_CAMERA) \
        .map(lambda msg: [v for v in msg.contours]) \
        .start_with([])

    def publish_map(map, scans):
        return scans.with_latest_from(odometry, lambda v, o: (v, o, rospy.get_rostime().to_sec())) \
            .pairwise() \
            .map(diff_state) \
            .do_action(map.update) \
            .subscribe(on_next=lambda _: map.publish(),
                       on_error=lambda e: rospy.logerr(traceback.format_exc(e)))

    # FIXME
    # def publish_lane_map(map, scans):
    #     return scans.with_latest_from(odometry, lambda v, o: (v, o, rospy.get_rostime().to_sec())) \
    #         .pairwise() \
    #         .map(diff_state) \
    #         .do_action(map.update) \
    #         .subscribe(on_next=lambda _: map.publish_lanes(),
    #                    on_error=lambda e: rospy.logerr(traceback.format_exc(e)))
    #     publish_lane_map(camera_map, no_barrels_camera)

    publish_map(combined_map, lidar.with_latest_from(camera, lambda l, c: l + c))

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

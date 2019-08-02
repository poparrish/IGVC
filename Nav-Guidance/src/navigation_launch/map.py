import math

import cv2
import numpy as np
import rospy
from breezyslam.algorithms import RMHC_SLAM
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Point, Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from std_msgs.msg import Header

import topics
from map_msg import MapData
from util.rosutil import get_rotation

UNKNOWN = 127  # unmapped/unknown pixel value used by BreezySLAM
EMPTY = 255  # no obstacles (white)


class MapUpdate:
    def __init__(self, scan, transform, time=None):
        self.scan = scan
        self.transform = transform
        self.time = time if time is not None else rospy.get_rostime().to_sec()


class Map:
    def __init__(self, size_px, size_meters, laser, max_travel=0.5, max_rotation=10):
        if laser.scan_size != laser.detection_angle_degrees:
            raise ValueError('laser scan_size must equal detection_angle_degrees')
        self.size_px = size_px
        self.size_meters = size_meters
        self.map_bytes = bytearray(size_px ** 2)
        self.laser = laser
        self.slam = self.create_slam()
        zero = Transform(rotation=Quaternion(0, 0, 0, 1))
        self.transform = zero  # absolute offset relative to world frame
        self.last_update = MapUpdate(scan=[], transform=zero)
        self.max_travel = max_travel
        self.max_rotation = 10

    def create_slam(self):
        return RMHC_SLAM(
            laser=self.laser,
            map_size_pixels=self.size_px,
            map_size_meters=self.size_meters,
            hole_width_mm=300,
            sigma_theta_degrees=0,  # disable directional prediction; we have a compass
            sigma_xy_mm=0)

    def update(self, map_update):
        transform = map_update.transform
        last_transform = self.last_update.transform

        # Calculate pose_change
        dx_mm = (transform.translation.x - last_transform.translation.x) * 1000.0
        dy_mm = (transform.translation.y - last_transform.translation.y) * 1000.0
        dxy_mm = (math.sqrt(dx_mm ** 2 + dy_mm ** 2))
        dtheta_degrees = math.degrees(transform.rotation.z - last_transform.rotation.z)
        dt_seconds = map_update.time - self.last_update.time

        scan = self.normalize_scan(map_update.scan)
        # if len(scan) != self.laser.scan_size:
        #     raise ValueError('len(scan) != laser.scan_size %d %d' % (len(scan), self.laser.scan_size))

        self.slam.update(scans_mm=[v.mag for v in scan],
                         scan_angles_degrees=[v.angle for v in scan],
                         pose_change=(dxy_mm, dtheta_degrees, dt_seconds))
        self.slam.getmap(self.map_bytes)
        self.last_update = map_update

        traveled = max(abs(self.transform.translation.x - transform.translation.x),
                       abs(self.transform.translation.y - transform.translation.y))
        rotated = abs(get_rotation(self.transform) - get_rotation(transform))
        if traveled > self.max_travel or rotated > self.max_rotation:
            self.reset()
            self.transform = map_update.transform

    def in_range(self, angle):
        detection = self.laser.detection_angle_degrees / 2.0
        return angle <= detection or angle > 360 - detection

    def normalize_scan(self, scan):
        closest = {}

        for v in scan:
            angle = int(v.angle)
            if self.in_range(angle) and (angle not in closest or closest[angle].mag > v.mag):
                closest[angle] = v

        scan = [v.with_angle(v.angle + self.laser.detection_angle_degrees / 2.0) for v in closest.values()]
        return sorted(scan, key=lambda v: v.angle)

    def reset(self):
        img = np.reshape(self.map_bytes, (self.size_px, self.size_px))
        rows, cols = img.shape

        position = self.slam.position

        border_mode = cv2.BORDER_CONSTANT
        border_value = (UNKNOWN)

        # rotate
        matrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), position.theta_degrees, 1)
        img = cv2.warpAffine(img, matrix, (cols, rows), borderMode=border_mode, borderValue=border_value)

        # translate'/
        matrix = np.float32([[1, 0, self.mm_to_pixels(position.x_mm)],
                             [0, 1, self.mm_to_pixels(position.y_mm)]])
        img = cv2.warpAffine(img, matrix, (cols, rows), borderMode=border_mode, borderValue=border_value)

        # reset slam
        self.map_bytes = bytearray(np.reshape(img, (self.size_px ** 2)))
        self.slam = self.create_slam()
        self.slam.map.set(self.map_bytes)

    #
    # Conversions
    #

    def mm_to_pixels(self, mm):
        m = self.size_meters / 2.0 - mm / 1000.0
        return int(m / self.size_meters * self.size_px)

    def to_img(self):
        img = np.array(self.map_bytes, dtype=np.uint8)
        for i in xrange(len(img)):
            if img[i] == UNKNOWN:
                img[i] = EMPTY
        img = np.reshape(img, (self.size_px, self.size_px, 1), order='F')
        img = np.rot90(img, 1)
        return img

    def to_msg(self, tf_frame):
        return MapData(map_data=self.to_img(),
                       transform=self.get_transform(tf_frame))

    def to_occupancy_grid(self):
        offset = self.size_meters / -2.0
        return OccupancyGrid(
            info=MapMetaData(resolution=float(self.size_meters) / self.size_px,
                             width=self.size_px,
                             height=self.size_px,
                             origin=Pose(position=Point(x=offset, y=offset))),
            data=[((255 - x) / 255.0 * 127.0 if x != UNKNOWN else -1) for x in self.map_bytes])

    def get_transform(self, tf_frame):
        return TransformStamped(
            header=Header(frame_id=topics.WORLD_FRAME,
                          stamp=rospy.Time.now()),
            child_frame_id=tf_frame,
            transform=self.transform)

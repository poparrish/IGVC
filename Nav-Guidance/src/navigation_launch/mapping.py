#!/usr/bin/env python
import math
import pickle
import time
import traceback
from collections import OrderedDict

import cv2
import numpy as np
import rospy
import tf
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, TransformStamped, Transform, Quaternion, Point32
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage

import topics
from map import Map, MapUpdate
from map_msg import MapData
from util import rx_subscribe, Vec2d, to360, to180
from util.rosutil import extract_tf

MAP_SIZE_PIXELS = 101
MAP_SIZE_METERS = 5

MIN_SAMPLES = 50
MAX_DIST_MM = 10000
MAX_TRAVEL_M = 0.5  # TODO: Name

UNKNOWN = 127  # unmapped/unknown value set in map

bridge = CvBridge()

point_cloud = rospy.Publisher('point_cloud', PointCloud, queue_size=10)


def condense_scan(scans):
    nearest = {}

    for scan in scans:
        angle = int(scan.angle)
        if angle not in nearest or nearest[angle].mag > scan.mag:
            nearest[angle] = scan

    return nearest.values()


def fill_scan(scans):
    scans = condense_scan(scans)
    scans = sorted(scans, key=lambda v: v.angle)

    if len(scans) <= 1:
        return []

    result = []
    group = []


    last = scans[0]

    def push_group():
        result.append(Vec2d(group[0].angle - 1, MAX_DIST_MM))
        result.extend(group)
        result.append(Vec2d(group[-1].angle + 1, MAX_DIST_MM))

    for curr in scans:
        if abs(curr.angle - last.angle) > 10:
            push_group()
            group = []
        group.append(curr)
        last = curr
    push_group()
    scans = condense_scan([Vec2d(a, MAX_DIST_MM) for a in xrange(360)] + result)
    scans = sorted(scans, key=lambda v: v.angle)
    print 'done %s' % min([v.mag for v in scans])

    # s = [Vec2d(angle, 1000) for angle in xrange(10)]
    # point_cloud.publish(PointCloud(header=Header(frame_id='map'),
    #                                points=[Point32(x=v.x / 1000.0, y=v.y / 1000.0) for v in scans]))
    return scans


def create_laser(detection_angle_degrees):
    return Laser(scan_size=detection_angle_degrees,
                 scan_rate_hz=5.5,
                 detection_angle_degrees=detection_angle_degrees,
                 distance_no_detection_mm=MAX_DIST_MM,
                 offset_mm=0)


class MapPublisher:
    def __init__(self, map, map_topic, tf_frame=None):
        self.map = map
        self.map_pub = rospy.Publisher(map_topic, String, queue_size=1)
        self.tf_frame = tf_frame
        if tf_frame is not None:
            self.br = tf.TransformBroadcaster()

        # debug
        self.rviz_pub = rospy.Publisher(map_topic + '_rviz', OccupancyGrid, queue_size=1)
        self.img_pub = rospy.Publisher(map_topic + '_img', Image, queue_size=1)
        self.point_cloud_pub = rospy.Publisher(map_topic + '_point_cloud', PointCloud, queue_size=10)

    def publish(self):
        start = time.time()
        msg = MapData(map_data=self.map.to_img(),
                      transform=self.map.get_transform(self.tf_frame))
        self.map_pub.publish(pickle.dumps(msg))
        self.publish_pose()
        self.rviz_pub.publish(self.map.to_occupancy_grid())
        self.img_pub.publish(bridge.cv2_to_imgmsg(msg.map_bytes, "mono8"))
        end = time.time()
        print 'Publish took: %s' % (end - start)

    def publish_pose(self):
        if hasattr(self, 'br'):
            self.br.sendTransformMessage(self.map.get_transform(self.tf_frame))

    def update(self, update):
        self.map.update(update)
        point_cloud.publish(PointCloud(header=Header(frame_id='map'),
                                       points=[Point32(x=v.x / 1000.0, y=v.y / 1000.0) for v in update.scan]))
        self.publish()

def start_mapping():
    rospy.init_node('mapping')

    camera_detection_angle = 120
    lidar_detection_angle = 270

    odometry = rx_subscribe('/tf', TFMessage, parse=None) \
        .let(extract_tf(topics.ODOMETRY_FRAME)) \
        .start_with(TransformStamped(transform=Transform(rotation=Quaternion(0, 0, 0, 1))))

    no_barrels_camera = rx_subscribe(topics.CAMERA) \
        .map(lambda msg: fill_scan(msg.contours)) \
        .start_with([])

    def publish_map(map_pub, scans):
        return scans.with_latest_from(odometry, lambda s, o: (s, o, rospy.get_rostime().to_sec())) \
            .throttle_last(200) \
            .subscribe(on_next=lambda (s, o, t): map_pub.update(MapUpdate(scan=s, time=t, transform=o)),
                       on_error=lambda e: rospy.logerr(traceback.format_exc(e)))

    # lane_map = Map(topics.LANE_MAP, detection_angle_degrees=camera_detection_angle)
    # update_map(lane_map, no_barrels_camera) \
    #     .subscribe(on_next=lambda cam: lane_map.publish(lane_map.to_message()),
    #                on_error=lambda e: rospy.logerr(traceback.format_exc(e)))

    camera_map = Map(size_px=MAP_SIZE_PIXELS,
                     size_meters=MAP_SIZE_METERS,
                     laser=create_laser(camera_detection_angle))
    camera_pub = MapPublisher(camera_map, topics.MAP, topics.MAP_FRAME)
    publish_map(camera_pub, no_barrels_camera)

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

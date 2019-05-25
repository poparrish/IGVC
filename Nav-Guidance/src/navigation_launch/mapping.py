#!/usr/bin/env python
import pickle
import time
import traceback

import rospy
import tf
from breezyslam.sensors import Laser
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Point32
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage

import topics
from map import Map, MapUpdate
from map_msg import MapData
from util import rx_subscribe, Vec2d
from util.rosutil import extract_tf

MAP_SIZE_PIXELS = 101
MAP_SIZE_METERS = 5

MIN_SAMPLES = 50
MAX_DIST_MM = 10000


class MapPublisher:
    def __init__(self, map, map_topic, tf_frame=None):
        self.map = map
        self.map_pub = rospy.Publisher(map_topic, String, queue_size=1)
        self.tf_frame = tf_frame
        if tf_frame is not None:
            self.br = tf.TransformBroadcaster()

        # debug
        self.bridge = CvBridge()
        self.rviz_pub = rospy.Publisher(map_topic + '_rviz', OccupancyGrid, queue_size=1)
        self.point_cloud_pub = rospy.Publisher(map_topic + '_point_cloud', PointCloud, queue_size=10)

    def publish(self):
        start = time.time()
        msg = MapData(map_data=self.map.to_img(),
                      transform=self.map.get_transform(self.tf_frame))
        self.map_pub.publish(pickle.dumps(msg))
        self.publish_pose()
        self.rviz_pub.publish(self.map.to_occupancy_grid())
        end = time.time()
        print 'Publish took: %s' % (end - start)

    def publish_pose(self):
        if hasattr(self, 'br'):
            self.br.sendTransformMessage(self.map.get_transform(self.tf_frame))

    def update(self, update):
        self.map.update(update)
        self.point_cloud_pub.publish(PointCloud(header=Header(frame_id='map'),
                                                points=[Point32(x=v.x / 1000.0, y=v.y / 1000.0) for v in update.scan]))
        self.publish()


def create_laser(detection_angle_degrees):
    return Laser(scan_size=detection_angle_degrees,
                 scan_rate_hz=5.5,
                 detection_angle_degrees=detection_angle_degrees,
                 distance_no_detection_mm=MAX_DIST_MM,
                 offset_mm=0)


def start_mapping():
    rospy.init_node('mapping')

    maxed = [Vec2d(a, MAX_DIST_MM) for a in xrange(360)]

    odometry = rx_subscribe('/tf', TFMessage, parse=None) \
        .let(extract_tf(topics.ODOMETRY_FRAME)) \
        .start_with(TransformStamped(transform=Transform(rotation=Quaternion(0, 0, 0, 1))))

    def publish_map(map_pub, scans):
        return scans.with_latest_from(odometry, lambda s, o: (s, o, rospy.get_rostime().to_sec())) \
            .throttle_last(200) \
            .subscribe(on_next=lambda (s, o, t): map_pub.update(MapUpdate(scan=s, time=t, transform=o)),
                       on_error=lambda e: rospy.logerr(traceback.format_exc(e)))

    # lidar
    lidar_map = Map(size_px=MAP_SIZE_PIXELS,
                    size_meters=MAP_SIZE_METERS,
                    laser=create_laser(detection_angle_degrees=270))
    lidar_data = rx_subscribe(topics.LIDAR) \
        .map(lambda scan: scan) \
        .start_with([])
    publish_map(MapPublisher(lidar_map, topics.MAP, tf_frame=topics.MAP_FRAME),
                scans=lidar_data)

    # camera
    camera_map = Map(size_px=MAP_SIZE_PIXELS,
                     size_meters=MAP_SIZE_METERS,
                     laser=create_laser(detection_angle_degrees=120))
    camera_data = rx_subscribe(topics.CAMERA) \
        .map(lambda msg: maxed + msg.contours) \
        .start_with([])
    publish_map(MapPublisher(camera_map, topics.LANE_MAP), camera_data)

    rospy.spin()


if __name__ == '__main__':
    start_mapping()

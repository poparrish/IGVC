import pickle
import time

import rospy
import tf
from breezyslam.sensors import Laser
from geometry_msgs.msg import Transform, Quaternion, Point32, Vector3
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, String

import topics
from map import MapUpdate, Map
from util import Vec2d
from util.rosutil import unpickle

MAP_SIZE_PIXELS = 101
MAP_SIZE_METERS = 5

MIN_SAMPLES = 50
MAX_DIST_MM = 10000

MAXED = [Vec2d(a, MAX_DIST_MM) for a in xrange(360)]


class MapPublisher:
    def __init__(self, map, map_topic, tf_frame=None):
        self.map = map
        self.map_pub = rospy.Publisher(map_topic, String, queue_size=1)
        self.tf_frame = tf_frame
        if tf_frame is not None:
            self.br = tf.TransformBroadcaster()

        # debug
        self.rviz_pub = rospy.Publisher(map_topic + '_rviz', OccupancyGrid, queue_size=1)
        self.point_cloud_pub = rospy.Publisher(map_topic + '_point_cloud', PointCloud, queue_size=10)

    def publish(self):
        start = time.time()
        msg = self.map.to_msg(self.tf_frame)
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


def create_laser(detection_angle_degrees, detection_margin=10):
    return Laser(scan_size=detection_angle_degrees,
                 scan_rate_hz=5.5,
                 detection_angle_degrees=detection_angle_degrees,
                 distance_no_detection_mm=MAX_DIST_MM,
                 offset_mm=0,
                 detection_margin=detection_margin)


def create_map(detection_angle_degrees, detection_margin=0):
    return Map(size_px=MAP_SIZE_PIXELS,
               size_meters=MAP_SIZE_METERS,
               laser=create_laser(detection_angle_degrees, detection_margin))


def publish_map(map_pub, topic, process_msg=lambda x: x):
    tf_listener = tf.TransformListener()

    def callback(scan):
        try:
            t, r = tf_listener.lookupTransform(topics.WORLD_FRAME, topics.ODOMETRY_FRAME, rospy.Time(0))
            transform = Transform(translation=Vector3(*t), rotation=Quaternion(*r))
        except Exception as e:
            rospy.logerr('Failed to lookup transform', e)
            transform = Transform(translation=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1))
        map_pub.update(MapUpdate(scan=process_msg(unpickle(scan)),
                                 transform=transform))

    rospy.Subscriber(topic, String, callback, queue_size=1)

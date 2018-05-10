import matplotlib.pyplot as plt
import pickle

import rospy
from rx.subjects import BehaviorSubject
from std_msgs.msg import String

from gps import GPSMsg, GPS_NODE
from lidar import MAX_DIST_CM, LIDAR_NODE
from lidar.avoidance import partition_scan, calculate_heading, ATTRACTOR_THRESHOLD_CM
from lidar.lidar import vectorize_scan
from scans import test_scans
from util.vec2d import Vec2d

DEBUG_NODE = 'NAV_DEBUG'
REFRESH_HZ = 15

lidar_data = BehaviorSubject([])
gps_data = BehaviorSubject(GPSMsg(0, 0, 0, False))


def gps_updated(data):
    msg = pickle.loads(data.data)
    rospy.loginfo("gps %s" % vars(msg))
    gps_data.on_next(msg)


def lidar_updated(data):
    msg = pickle.loads(data.data)
    lidar_data.on_next(msg)


def draw_plot(scan, goal):
    plt.clf()

    # Lidar data
    # scan = [v.with_angle(v.angle - 90) for v in scan]
    plt.scatter(x=[v.x for v in scan],
                y=[v.y for v in scan],
                s=[10 for _ in scan])

    cluster = partition_scan(scan)
    plt.scatter(x=[v.x for (v, _) in cluster],
                y=[v.y for (v, _) in cluster],
                s=[10 * c for (_, c) in cluster])

    # Robot
    plt.scatter([0], [0], s=[100], c=[(1, 0, 0)])

    # Heading
    heading = calculate_heading(scan, goal)
    plt.quiver([0, goal.x], [0, goal.y], color='g')
    plt.quiver([0, heading.x], [0, heading.y])

    axis_range = [-MAX_DIST_CM, MAX_DIST_CM]
    plt.ylim(axis_range)
    plt.xlim(axis_range)


def debug_start():
    rospy.init_node(DEBUG_NODE)
    rospy.Subscriber(GPS_NODE, String, gps_updated)
    rospy.Subscriber(LIDAR_NODE, String, lidar_updated)

    # Generate test data
    goal = Vec2d(0, ATTRACTOR_THRESHOLD_CM * 2)

    # Scale up test data
    s = [[v * 2.5 for v in vectorize_scan(s)] for s in test_scans]

    i = 0
    while not rospy.is_shutdown():
        i = (i + 1) % len(s)
        draw_plot(s[i], goal)
        plt.pause(1.0 / REFRESH_HZ)


if __name__ == '__main__':
    debug_start()

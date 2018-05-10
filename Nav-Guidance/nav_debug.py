import matplotlib.pyplot as plt
import pickle

import rospy
from rx.subjects import BehaviorSubject
from std_msgs.msg import String

from gps import GPSMsg, GPS_NODE
from lidar import MAX_DIST_MM, LIDAR_NODE
from lidar.avoidance import partition_scan, calculate_heading, ATTRACTOR_THRESHOLD_MM
from lidar.lidar import vectorize_scan
from scans import test_scans
from util.vec2d import Vec2d

DEBUG_NODE = 'NAV_DEBUG'
REFRESH_HZ = 15

lidar_data = BehaviorSubject([])
gps_data = BehaviorSubject(GPSMsg(0, 0, 0, False))

dest = (43.600532, -116.200390)  # rec center


def gps_updated(data):
    msg = pickle.loads(data.data)
    gps_data.on_next(msg)


def lidar_updated(data):
    msg = pickle.loads(data.data)
    lidar_data.on_next(msg)


def write(ser, wheel, wheel_theta, wheel_speed):
    ser.write('W%sB%sS%s\n' % (int(wheel), wheel_theta, wheel_speed))


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
                s=[10 * c for (_, c) in cluster],
                c='purple')

    # Robot
    plt.scatter([0], [0], s=[100], c=[(1, 0, 0)])

    # Heading
    heading = calculate_heading(scan, goal)
    plt.quiver([0, goal.x], [0, goal.y], color='grey')
    plt.quiver([0, heading.x], [0, heading.y])

    axis_range = [-MAX_DIST_MM, MAX_DIST_MM]
    plt.ylim(axis_range)
    plt.xlim(axis_range)


def debug_start():
    # ser = serial.Serial('/dev/ttyACM0', 115200)

    rospy.init_node(DEBUG_NODE)
    rospy.Subscriber(GPS_NODE, String, gps_updated)
    rospy.Subscriber(LIDAR_NODE, String, lidar_updated)

    # Generate test data
    goal = Vec2d(0, ATTRACTOR_THRESHOLD_MM * 2)

    # Scale up test data
    s = [[v * 1.5 for v in vectorize_scan(s)] for s in test_scans]

    i = 0
    while not rospy.is_shutdown():
        i = (i + 1) % len(s)

        current_scan = s[i]
        # current_scan = lidar_data.value
        # gps_location = gps_data.value
        # if current_scan is None or gps_location.fixed is False:
        #     print("none")
        #     time.sleep(0.1)
        #     continue

        # goal = calculate_gps_heading(gps_location, dest)

        draw_plot(current_scan, goal)

        heading = calculate_heading(current_scan, goal)
        theta = heading.angle
        if theta > 180:
            theta = -(360 - theta)
        # print("theta = %s" % theta)
        # print("goal = %s" % goal.angle)

        theta = max(-90, min(theta, 90))
        print("theta = " + str(theta))

        # for w in range(4):
        #     write(ser, w, theta, 60)

        plt.pause(1.0 / REFRESH_HZ)


if __name__ == '__main__':
    debug_start()

import itertools
import matplotlib.pyplot as plt
import pickle
from matplotlib import cm

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from rx.subjects import BehaviorSubject
from std_msgs.msg import String

from camera_msg import CameraMsg
from cameras import CAMERA_NODE
from gps import GPSMsg, GPS_NODE
from guidance import ATTRACTOR_THRESHOLD_MM, calculate_potential, contours_to_vectors, calculate_line_angle
from lidar import convert_scan_to_vectors, MAX_DIST_MM, LIDAR_NODE
from scans import test_scans
from util import Vec2d

DEBUG_NODE = 'NAV_DEBUG'
REFRESH_HZ = 8

lidar_data = BehaviorSubject([])
gps_data = BehaviorSubject(GPSMsg(0, 0, 0, False))
camera_data = BehaviorSubject([])

dest = (43.600532, -116.200390)  # rec center

SIMULATE = True
SPEED = 0.5


def gps_updated(data):
    msg = pickle.loads(data.data)
    gps_data.on_next(msg)


def lidar_updated(data):
    msg = pickle.loads(data.data)
    lidar_data.on_next(msg)


def camera_updated(data):
    msg = CameraMsg(pickled_values=data.data)
    camera_data.on_next(contours_to_vectors(msg.contours))


def plot_vectors(vecs, size=None, color=None):
    plt.scatter(x=[v.x for v in vecs],
                y=[v.y for v in vecs],
                s=size,
                c=color)


def draw_plot(lidar, camera, goal, heading, rotation):
    plt.clf()

    # Lidar data
    # scan = [v.with_angle(v.angle - 90) for v in scan]
    plot_vectors(lidar, size=10, color='blue')
    plot_vectors(camera, size=10, color='green')

    # Robot
    plt.scatter([0], [0], s=[100], c=[(1, 0, 0)])

    # Heading
    plt.quiver([0, goal.x], [0, goal.y], color='grey')
    plt.quiver([0, heading.x], [0, heading.y])
    rotation = Vec2d(rotation, 1)
    plt.quiver([0, rotation.x], [0, rotation.y], color='blue')

    axis_range = [-MAX_DIST_MM, MAX_DIST_MM]
    plt.ylim(axis_range)
    plt.xlim(axis_range)


res_3d = 200
fig_3d = plt.figure()


def to_mm(i):
    return i * res_3d - MAX_DIST_MM


def replicate(item, times):
    return list(itertools.repeat(item, times))


def draw_plot_3d(lidar, camera, goal):
    plt.clf()
    ax = fig_3d.add_subplot(111, projection='3d')

    dim = MAX_DIST_MM * 2 / res_3d + 1
    r = xrange(0, dim)

    x = [replicate(to_mm(i), times=dim) for i in r]
    y = replicate(map(to_mm, r), times=dim)

    # take the cartesian product
    ms = [to_mm(i) for i in r]
    # z_old = [Vec2d.from_point(x, y) in itertools.product(ms, ms)]

    z = []
    for i in ms:
        row = []
        for j in ms:
            row.append(calculate_potential(lidar, camera, goal, position=Vec2d.from_point(i, j)).mag)
        z.append(row)

    ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


def start_debug():
    rospy.init_node(DEBUG_NODE)
    pub = rospy.Publisher('control', numpy_msg(Floats), queue_size=3)
    rospy.Subscriber(GPS_NODE, String, gps_updated)
    rospy.Subscriber(LIDAR_NODE, String, lidar_updated)
    rospy.Subscriber(CAMERA_NODE, String, camera_updated)

    # Generate test data
    goal = Vec2d(0, ATTRACTOR_THRESHOLD_MM)

    # Scale up test data
    s = [[v * 1 for v in convert_scan_to_vectors(s)] for s in test_scans]

    i = 0
    rate = rospy.Rate(REFRESH_HZ)
    while not rospy.is_shutdown():
        i = (i + 1) % len(s)

        # current_scan = lidar_data.value
        # gps_location = gps_data.value
        # if current_scan is None or gps_location.fixed is False:
        #     print("none")
        #     time.sleep(0.1)
        #     continue

        # goal = calculate_gps_heading(gps_location, dest)

        # TODO: REPLAY INSTEAD
        lidar = s[i] if SIMULATE else lidar_data.value
        camera = camera_data.value
        camera_flat = [v for c in camera for v in c]

        desired_heading = calculate_potential(lidar, camera_flat, goal)

        # calculate translational theta
        translation = desired_heading.angle
        if translation > 180:
            translation = -(360 - translation)
        translation = max(-90, min(translation, 90))
        heading = desired_heading.with_angle(translation)

        theta_dot = calculate_line_angle(camera)
        if abs(theta_dot) < 10:
            theta_dot = 0
        theta_dot /= 4.0

        # draw_plot_3d(lidar, camera, goal)

        print("translation = %s" % translation)

        a = np.array([translation, theta_dot, SPEED], dtype=np.float32)
        # a = np.array([4, 12, 0], dtype=np.float32)
        pub.publish(a)

        draw_plot(lidar, camera_flat, goal, heading, theta_dot)
        plt.pause(1.0 / REFRESH_HZ)


if __name__ == '__main__':
    start_debug()

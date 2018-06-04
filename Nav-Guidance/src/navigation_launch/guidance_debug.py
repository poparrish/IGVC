import matplotlib.pyplot as plt

import rospy
from rx.subjects import ReplaySubject

from lidar import MAX_DIST_MM
from util import Vec2d, rx_subscribe

DEBUG_NODE = 'NAV_DEBUG'
REFRESH_HZ = 8

dest = (43.600532, -116.200390)  # rec center

SIMULATE = True
SPEED = 0.5

data = None


def msg_received(msg):
    global data
    data = msg


def plot_vectors(vecs, size=None, color=None):
    xs = [v.x for v in vecs]
    ys = [v.y for v in vecs]

    xsf = ys
    ysf = ys
    plt.scatter(x= xs,
                y= ys,
                s=size,
                c=color)


def draw_plot(data):
    plt.clf()

    lidar = data['lidar']
    camera = [v for c in data['camera'] for v in c]
    gps = data['gps']
    translation = data['translation']
    rotation = data['rotation']
    goal = data['goal']

    # Lidar data
    # scan = [v.with_angle(v.angle - 90) for v in scan]
    plot_vectors(lidar, size=10, color='blue')
    plot_vectors(camera, size=10, color='green')

    # Robot
    plt.scatter([0], [0], s=[100], c=[(1, 0, 0)])

    # Heading
    plt.quiver([0, goal.x], [0, goal.y], color='grey')
    translation = Vec2d(translation, 1)
    plt.quiver([0, translation.x], [0, translation.y])
    rotation = Vec2d(rotation, 1)
    plt.quiver([0, rotation.x], [0, rotation.y], color='pink')

    axis_range = [-MAX_DIST_MM, MAX_DIST_MM]
    plt.ylim(axis_range)
    plt.xlim(axis_range)


def start_debug():
    rospy.init_node(DEBUG_NODE)
    rx_subscribe('debug').subscribe(msg_received)

    while not rospy.is_shutdown():
        if data is not None:
            draw_plot(data)
        plt.pause(1.0 / REFRESH_HZ)


if __name__ == '__main__':
    start_debug()

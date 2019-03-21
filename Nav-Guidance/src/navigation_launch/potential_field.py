#!/usr/bin/env python
import math
import pickle

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler

import topics
from guidance.potential_field import calc_repulsive_force, ATTRACTOR_THRESHOLD_MM
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from util import rx_subscribe, Vec2d


def trace_ray(grid, x, y, prob_threshold, angle):
    x_inc = math.cos(math.radians(angle))
    y_inc = math.sin(math.radians(angle))

    start_x = x
    start_y = y

    data = grid.data
    while 0 <= int(x) < MAP_SIZE_PIXELS and 0 <= int(y) < MAP_SIZE_PIXELS:
        if data[int(x) + int(y) * MAP_SIZE_PIXELS] >= prob_threshold:
            return math.sqrt((x - start_x) ** 2 + (y - start_y) ** 2)
        x += x_inc
        y += y_inc

    return None


def extract_repulsors(pose, grid):
    scale = (MAP_SIZE_PIXELS / 2.0) / (MAP_SIZE_METERS / 2.0)
    x = int((MAP_SIZE_PIXELS / 2.0) - pose.x * scale)
    y = int((MAP_SIZE_PIXELS / 2.0) - pose.y * scale)

    repulsors = []
    for i in xrange(360):
        ray = trace_ray(grid, x, y, 80, i)
        if ray is not None:
            v = Vec2d(360 - i, ray * 1000.0 / MAP_SIZE_PIXELS * MAP_SIZE_METERS)
            repulsors.append(v)

    return repulsors


def compute_potential(pose_stamped, grid):
    pose = pose_stamped.pose.position
    pose = Vec2d.from_point(pose.x, pose.y)

    repulsors = extract_repulsors(pose, grid)
    r = sum([calc_repulsive_force(r, pose, 1) for r in repulsors])
    a = Vec2d(0, 10.0)
    p = r + a
    q = quaternion_from_euler(0, 0, math.radians(p.angle))

    return PoseStamped(header=Header(frame_id='map'),
                       pose=Pose(position=Point(x=pose.x, y=pose.y),
                                 orientation=Quaternion(q[0], q[1], q[2], q[3])))


def start():
    rospy.init_node('potential_field')
    pub = rospy.Publisher(topics.POTENTIAL_FIELD, PoseStamped, queue_size=1)

    grid = rx_subscribe(topics.MAP, OccupancyGrid, parse=None)
    pose = rx_subscribe(topics.MAP_POSE, PoseStamped, parse=None)
    pose.with_latest_from(grid, compute_potential) \
        .subscribe(pub.publish)

    rospy.spin()


if __name__ == '__main__':
    start()

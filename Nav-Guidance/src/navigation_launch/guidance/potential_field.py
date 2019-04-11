import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

import topics
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from util import Vec2d, rx_subscribe, avg

ATTRACTOR_THRESHOLD_MM = 1500
REPULSOR_THRESHOLD_MM = 1000

R_FACTOR = 1
A_FACTOR = 1

NOISE_THRESHOLD = 3


def calc_attractive_force(attractor, position):
    attractor -= position

    mag = min(attractor.mag, ATTRACTOR_THRESHOLD_MM)  # don't change scaling unless we're really close
    f = 0.5 * A_FACTOR * (attractor.mag / REPULSOR_THRESHOLD_MM) ** 2  # quadratic
    # f = 0.5 * A_FACTOR * mag ** 2  # quadratic
    return attractor.with_magnitude(f)


def calc_repulsive_force(repulsor, position, weight):
    repulsor -= position
    if repulsor.mag <= REPULSOR_THRESHOLD_MM:
        f = 0.5 * R_FACTOR * (repulsor.mag / REPULSOR_THRESHOLD_MM) ** 2  # quadratic
    else:
        f = 0  # out of range

    return repulsor.with_magnitude(f * weight)


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


def compute_potential(pose_stamped, grid, goal):
    pose = pose_stamped.pose.position
    pose = Vec2d.from_point(pose.x, pose.y)

    repulsors = extract_repulsors(pose, grid)
    r = sum([calc_repulsive_force(r, pose, 1) for r in repulsors])
    r = r.with_magnitude(min(1.0, r.mag))  # cap magnitude to something reasonable
    return r + goal


def start():
    rospy.init_node('potential_field')
    pub = rospy.Publisher(topics.POTENTIAL_FIELD, PoseStamped, queue_size=1)

    grid = rx_subscribe(topics.MAP, OccupancyGrid, parse=None)
    pose = rx_subscribe(topics.MAP_POSE, PoseStamped, parse=None)
    pose.with_latest_from(grid, compute_potential) \
        .subscribe(pub.publish)

    rospy.spin()

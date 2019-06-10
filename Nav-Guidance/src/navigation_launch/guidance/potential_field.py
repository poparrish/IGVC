import math

from costmap import COSTMAP_DILATION_M
from map import UNKNOWN
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from util import Vec2d, avg

ATTRACTOR_THRESHOLD_MM = 1500
REPULSOR_THRESHOLD_MM = 1500

R_FACTOR = 1.0 / (400 ** 2)  # 750 is the distance at which the repulsors should start to overpower the attractors
A_FACTOR = 25.0 / (1500 ** 2)  # dumb hack to ensure 25

NOISE_THRESHOLD = 5


def partition(vecs, cluster_mm):
    """groups a list of vectors into clusters"""
    if len(vecs) == 0:
        return []

    vecs = sorted(vecs, key=lambda v: v.angle)

    prev = vecs[0]
    group = [prev]
    groups = []
    for i in range(1, len(vecs)):
        curr = vecs[i]
        if (prev - curr).mag < cluster_mm:
            group.append(curr)
        else:
            # if a cluster doesn't have at least 5 readings, we'll consider it noise
            if len(group) > NOISE_THRESHOLD:
                groups.append(group)
            group = [curr]
            prev = curr
    groups.append(group)

    return map(avg, groups)


def calc_attractive_force(attractor, position):
    attractor -= position

    mag = min(attractor.mag, ATTRACTOR_THRESHOLD_MM)  # don't change scaling unless we're really close
    f = 0.5 * A_FACTOR * mag ** 2  # quadratic
    return attractor.with_magnitude(f)


def calc_repulsive_force(repulsor, position, weight):
    repulsor -= position
    if repulsor.mag <= 200:
        f = 0.5 * R_FACTOR * (REPULSOR_THRESHOLD_MM - repulsor.mag) ** 2  # quadratic
    elif repulsor.mag <= 1000:
        f = 0.5 * 400 * R_FACTOR * (REPULSOR_THRESHOLD_MM - repulsor.mag)
    else:
        f = 0  # out of range

    return repulsor.with_magnitude(f * weight)


zero = Vec2d(0, 0)


def sum_repulsors(vecs, position, cluster_mm, weight):
    if len(vecs) == 0:
        return zero

    clusters = partition(vecs, cluster_mm)
    return sum([calc_repulsive_force(r, position, weight) for r in clusters])


def compute_potential(repulsors, goal, position=zero):
    a = calc_attractive_force(goal, position)
    r = sum_repulsors(repulsors, zero, cluster_mm=150, weight=2)
    print a, r
    return a - r


def trace_ray(grid, x, y, prob_threshold, angle):
    x_inc = math.cos(math.radians(angle))
    y_inc = math.sin(math.radians(angle))

    start_x = x
    start_y = y

    while 0 <= int(x) < MAP_SIZE_PIXELS and 0 <= int(y) < MAP_SIZE_PIXELS:
        val = grid[int(y)][int(x)]
        if val < prob_threshold and val != UNKNOWN:
            return math.sqrt((x - start_x) ** 2 + (y - start_y) ** 2)
        x += x_inc
        y -= y_inc

    return None


def extract_repulsors((x, y), grid):
    pose = Vec2d.from_point(x, y)
    scale = (MAP_SIZE_PIXELS / 2.0) / (MAP_SIZE_METERS / 2.0)
    x = int((MAP_SIZE_PIXELS / 2.0) + pose.x * scale)
    y = int((MAP_SIZE_PIXELS / 2.0) - pose.y * scale)

    repulsors = []
    for i in xrange(360):
        ray = trace_ray(grid, x, y, 80, i)
        if ray is not None:
            v = Vec2d(i, ray * 1000.0 / MAP_SIZE_PIXELS * MAP_SIZE_METERS)
            repulsors.append(v)

    return repulsors

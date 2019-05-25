import math

from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from util import Vec2d, avg

ATTRACTOR_THRESHOLD_MM = 1500
REPULSOR_THRESHOLD_MM = 1000
R_GAIN = 1 / 1000.0

NOISE_THRESHOLD = 3


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


def sum_repulsors(vecs, cluster_mm, weight):
    if len(vecs) == 0:
        return Vec2d(0, 0)

    clusters = partition(vecs, cluster_mm)
    return avg([calc_repulsive_force(r, weight) for r in clusters])


def calc_repulsive_force(repulsor, weight):
    if repulsor.mag <= REPULSOR_THRESHOLD_MM:
        f = 0.5 * R_GAIN * (REPULSOR_THRESHOLD_MM - repulsor.mag) ** 2  # quadratic
        # f = R_GAIN * (1.0 / REPULSOR_THRESHOLD_MM - 1.0 / repulsor.mag) * 1.0 / (repulsor.mag ** 2)
        return repulsor.with_magnitude(abs(f * weight))
    else:
        f = 0  # out of range
        return repulsor.with_magnitude(0)


def trace_ray(grid, x, y, prob_threshold, angle):
    x_inc = math.cos(math.radians(angle))
    y_inc = math.sin(math.radians(angle))

    start_x = x
    start_y = y

    while 0 <= int(x) < MAP_SIZE_PIXELS and 0 <= int(y) < MAP_SIZE_PIXELS:
        if grid[int(y)][int(x)] < prob_threshold:
            return math.sqrt((x - start_x) ** 2 + (y - start_y) ** 2)
        x += x_inc
        y -= y_inc

    return None


def extract_repulsors((x, y), grid):
    pose = Vec2d.from_point(x, y)
    scale = (MAP_SIZE_PIXELS / 2.0) / (MAP_SIZE_METERS / 2.0)
    x = int((MAP_SIZE_PIXELS / 2.0) - pose.x * scale)
    y = int((MAP_SIZE_PIXELS / 2.0) - pose.y * scale)

    repulsors = []
    for i in xrange(360):
        ray = trace_ray(grid, x, y, 80, i)
        if ray is not None:
            v = Vec2d(i, ray * 1000.0 / MAP_SIZE_PIXELS * MAP_SIZE_METERS)
            repulsors.append(v)

    return repulsors


def compute_potential(repulsors, goal):
    r = -sum_repulsors(repulsors, cluster_mm=150, weight=2)
    # r = r.with_magnitude(min(10.0, r.mag))  # cap magnitude to something reasonable
    print goal, r
    return goal + r

    # return goal

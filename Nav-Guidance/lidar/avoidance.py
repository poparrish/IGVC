import math

from gpxpy.geo import haversine_distance

from util import Vec2d

A_FACTOR = 1 / (10.0 ** 3)
R_FACTOR = 1 / (10.0 ** 5)

ATTRACTOR_THRESHOLD_MM = 2000
REPULSOR_THRESHOLD_MM = 2000

CLUSTER_MM = 500
NOISE_THRESHOLD = 5


def avg(coll):
    return sum(coll) / len(coll)


def partition_scan(v):
    """partition a scan into clusters"""
    if len(v) == 0:
        return []

    prev = v[0]
    group = [prev]
    groups = [group]
    for i in range(1, len(v)):
        curr = v[i]
        if (prev - curr).mag < CLUSTER_MM:
            group.append(curr)
        else:
            # if a cluster doesn't have at least 5 readings, we'll consider it noise
            if len(group) > NOISE_THRESHOLD:
                groups.append(group)
            group = [curr]
            prev = curr
    groups.append(group)

    return map(avg, groups)


def calc_attractive_force(at):
    if at.mag <= ATTRACTOR_THRESHOLD_MM:
        return 0.5 * A_FACTOR * at.mag ** 2  # quadratic
    else:
        return A_FACTOR * at.mag  # conical


def calc_repulsive_force(rp):
    if rp.mag <= REPULSOR_THRESHOLD_MM:
        return 0.5 * R_FACTOR * (REPULSOR_THRESHOLD_MM - rp.mag) ** 2  # quadratic
    else:
        return 0  # out of range


def calculate_heading(repulsors, goal):
    # attractors
    a = goal.with_magnitude(calc_attractive_force(goal))

    if len(repulsors) == 0:
        return a

    # repulsors
    clusters = partition_scan(repulsors)
    r = [rp.with_magnitude(calc_repulsive_force(rp)) for rp in clusters]
    r = avg(r)

    return a - r


# https://www.movable-type.co.uk/scripts/latlong.html
def initial_bearing((lat1, lon1), (lat2, lon2)):
    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - \
        math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


def calculate_gps_heading(loc, dest):
    """returns a vector with the initial bearing and distance between two points"""

    dist = haversine_distance(loc.lat, loc.lon, dest[0], dest[1])
    angle = initial_bearing((loc.lat, loc.lon), dest) - loc.heading

    return Vec2d(angle, dist)

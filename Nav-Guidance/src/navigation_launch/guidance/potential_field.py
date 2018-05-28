from util import Vec2d, avg

ATTRACTOR_THRESHOLD_MM = 1500
REPULSOR_THRESHOLD_MM = 1500

R_FACTOR = 1.0 / (500 ** 2)  # 750 is the distance at which the repulsors should start to overpower the attractors
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
    if repulsor.mag <= REPULSOR_THRESHOLD_MM:
        f = 0.5 * R_FACTOR * (REPULSOR_THRESHOLD_MM - repulsor.mag) ** 2  # quadratic
    else:
        f = 0  # out of range

    return repulsor.with_magnitude(f * weight)


zero = Vec2d(0, 0)


def sum_repulsors(vecs, position, cluster_mm, weight):
    if len(vecs) == 0:
        return zero

    clusters = partition(vecs, cluster_mm)
    return sum([calc_repulsive_force(r, position, weight) for r in clusters])


def calculate_potential(lidar_data, camera_data, goal, position=zero):
    a = calc_attractive_force(goal, position)

    rl = sum_repulsors(lidar_data, position, cluster_mm=150, weight=2)
    rc = sum_repulsors([v for c in camera_data for v in c], position, cluster_mm=500, weight=1)

    return a - rl - rc

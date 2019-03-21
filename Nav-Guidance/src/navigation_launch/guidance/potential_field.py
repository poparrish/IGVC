from util import Vec2d, avg

ATTRACTOR_THRESHOLD_MM = 1500
REPULSOR_THRESHOLD_MM = 1500

R_FACTOR = 1
A_FACTOR = 1

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


def calc_attractive_force(attractor, position):
    attractor -= position

    mag = min(attractor.mag, ATTRACTOR_THRESHOLD_MM)  # don't change scaling unless we're really close
    f = 0.5 * A_FACTOR * mag ** 2  # quadratic
    return attractor.with_magnitude(f)


def calc_repulsive_force(repulsor, position, weight):
    repulsor -= position
    if repulsor.mag <= REPULSOR_THRESHOLD_MM:
        f = 0.5 * R_FACTOR * (repulsor.mag / REPULSOR_THRESHOLD_MM) ** 2  # quadratic
    else:
        f = 0  # out of range

    return repulsor.with_magnitude(f * weight)


zero = Vec2d(0, 0)


def sum_repulsors(vecs, position, cluster_mm, weight):
    if len(vecs) == 0:
        return zero

    clusters = partition(vecs, cluster_mm)
    return sum([calc_repulsive_force(r, position, weight) for r in clusters])


def mask_lidar_in_cam(lidar_vec, cam_vec, tol):
    to_remove = set()
    for l_v in lidar_vec:
        for i in xrange(len(cam_vec)):
            c_v = cam_vec[i]
            if (c_v.x - tol < l_v.x < c_v.x + tol) and (c_v.y - tol < l_v.y < c_v.y + tol):
                to_remove.add(i)
    for ind in sorted(to_remove, reverse=True):
        del cam_vec[ind]
    return cam_vec


def calculate_potential(path, goal, position=zero):
    vecs = [Vec2d.from_point(p.pose.position.x, p.pose.position.y) for p in path.poses[:10]]
    vecs = [v.with_magnitude(1) for v in vecs]

    if len(vecs) > 0:
        return avg(vecs)
    else:
        return Vec2d(0, 0)

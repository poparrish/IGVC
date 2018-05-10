from util import Vec2d

A_FACTOR = 1 / (10.0 ** 3)
R_FACTOR = 1 / (10.0 ** 4)

ATTRACTOR_THRESHOLD_CM = 2500
REPULSOR_THRESHOLD_CM = 2500

CLUSTER_CM = 500


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
        if (prev - curr).mag < CLUSTER_CM:
            group.append(curr)
        else:
            groups.append(group)
            group = [curr]
            prev = curr
    groups.append(group)

    return [(Vec2d.from_point(x=avg([v.x for v in g]),
                              y=avg([v.y for v in g])),
             len(g)) for g in groups]


def calc_attractive_force(at):
    if at.mag <= ATTRACTOR_THRESHOLD_CM:
        return 0.5 * A_FACTOR * at.mag ** 2  # quadratic
    else:
        return A_FACTOR * at.mag  # conical


def calc_repulsive_force(rp):
    if rp.mag <= REPULSOR_THRESHOLD_CM:
        return 0.5 * R_FACTOR * (REPULSOR_THRESHOLD_CM - rp.mag) ** 2  # quadratic
    else:
        return 0  # out of range


def calculate_heading(repulsors, goal):
    a = goal.with_magnitude(calc_attractive_force(goal))

    if len(repulsors) == 0:
        return a

    clusters = partition_scan(repulsors)
    r = [rp.with_magnitude(calc_repulsive_force(rp)) for (rp, count) in clusters]
    r = reduce(lambda r1, r2: r1 + r2, r)
    r.mag /= len(clusters)

    return a - r

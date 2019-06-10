import math
import time

import rospy
from sensor_msgs.msg import PointCloud

from a_star import find_path, grid_neighbors, diagonal
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from util import Vec2d, to360, to180

SPACING = 5
CIRCLE_RADIUS = math.sqrt(2 * MAP_SIZE_PIXELS ** 2)


def edge_point():
    for x in xrange(MAP_SIZE_PIXELS + 1):
        yield (x, 0)
        yield (x, MAP_SIZE_PIXELS - 1)
    for y in xrange(MAP_SIZE_PIXELS + 1):
        yield (0, y)
        yield (MAP_SIZE_PIXELS - 1, y)


def navigable_edge_point((x, y), heading):
    angle = math.degrees(math.atan2(y - MAP_SIZE_PIXELS / 2.0,
                                    x - MAP_SIZE_PIXELS / 2.0))
    return abs(to180(angle)) <= 90


def x_to_pixel(m):
    return int(m / MAP_SIZE_METERS * MAP_SIZE_PIXELS + MAP_SIZE_PIXELS / 2.0)


def y_to_pixel(m):
    return int(-m / MAP_SIZE_METERS * MAP_SIZE_PIXELS + MAP_SIZE_PIXELS / 2.0)


debug = rospy.Publisher('/tmp2', PointCloud, queue_size=1)


def generate_path(costmap, heading, (x, y)):
    heading = to360(heading)
    valid_edges = set([e for e in edge_point() if navigable_edge_point(e, heading)])

    x = int(round(x_to_pixel(x) / float(SPACING))) * SPACING
    y = int(round(y_to_pixel(y) / float(SPACING))) * SPACING
    ideal = Vec2d.from_point(100, y)

    print 'ideal %s' % (ideal,)

    def is_goal(p):
        return p in valid_edges

    def weight((x, y)):
        # TODO: Figure out why this is needed...
        if not 0 < x < len(costmap) or not 0 < y < len(costmap):
            return float('inf')

        if costmap[y][x] < 127:
            # can't traverse
            return float('inf')

        if is_goal((x, y)):
            return abs(ideal.x - x) + abs(ideal.y - y)

        if abs(x - ideal.x) == abs(y - ideal.y):
            return 1.414
        return 1

    path = find_path(start=(x, y),
                     reached_goal=is_goal,
                     neighbors=grid_neighbors(costmap, jump_size=SPACING),
                     weight=weight,
                     heuristic=lambda v: diagonal(v, (ideal.x, ideal.y)))

    return path

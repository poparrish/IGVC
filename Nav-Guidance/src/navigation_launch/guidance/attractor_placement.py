# fixed circle, sqrt(w/2) in diameter
# weight points across edge every SPACING points based on circle radius
# add arbitrary, linear weight to each point based on dist from ideal
# this breaks created by using circe radius

# also, probably use Dijkstra's since there's no meaningful heuristic we can use

import math

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header

import topics
from a_star import find_path, grid_neighbors, euclidean, manhattan
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from util import Vec2d, to360, to180

SPACING = 1
CIRCLE_RADIUS = math.sqrt(2 * MAP_SIZE_PIXELS ** 2)


def edge_point():
    for x in xrange(MAP_SIZE_PIXELS / SPACING + 1):
        yield (x * SPACING, 0)
        yield (x * SPACING, MAP_SIZE_PIXELS - 1)
    for y in xrange(MAP_SIZE_PIXELS / SPACING + 1):
        yield (0, y * SPACING)
        yield (MAP_SIZE_PIXELS - 1, y * SPACING)


def navigable_edge_point((x, y), heading):
    angle = math.degrees(math.atan2(y - MAP_SIZE_PIXELS / 2.0,
                                    x - MAP_SIZE_PIXELS / 2.0))
    return abs(to180(angle)) <= 90


def x_to_pixel(m):
    return int(m / MAP_SIZE_METERS * MAP_SIZE_PIXELS + MAP_SIZE_PIXELS / 2.0)


def y_to_pixel(m):
    return int(-m / MAP_SIZE_METERS * MAP_SIZE_PIXELS + MAP_SIZE_PIXELS / 2.0)


debug = rospy.Publisher('/tmp2', PointCloud, queue_size=1)


def find_ideal(heading, x, y):
    heading = to360(heading)
    x = y = 50
    if 0 <= heading <= 45 or 315 <= heading <= 360:
        a = 50
        angle = math.cos(math.radians(heading))
    elif 45 <= heading <= 135:
        a = 50
        angle = math.cos(math.radians(heading-90))
    elif 135 <= heading <= 225:
        a = 50
        angle = math.cos(math.radians(heading-180))
    else:
        a = 50
        angle = math.cos(math.radians(heading-270))
    return Vec2d(heading, a / angle)

def generate_path(costmap, heading, (x, y)):
    heading = to360(heading)
    valid_edges = set([e for e in edge_point() if navigable_edge_point(e, heading)])

    x = int(round(x_to_pixel(x) / float(SPACING))) * SPACING
    y = int(round(y_to_pixel(y) / float(SPACING))) * SPACING
    ideal = (100, y)
    # xrange_ = [Point32(x=find_ideal(v, x, y).x / 50*2.5, y=find_ideal(v, x, y).y / 50*2.5) for v in xrange(360)]
    # xrange_ = [Point32(x=find_ideal(v, x, y).x / 50*2.5, y=find_ideal(v, x, y).y / 50*2.5) for v in [heading]]
    xrange_ = [Point32(x=(v[0] - 50) / 50.0 * 2.5, y=(v[1] - 50) / 50.0 * 2.5) for v in valid_edges]
    debug.publish(PointCloud(header=Header(frame_id=topics.MAP_FRAME),
                             points=xrange_))

    print 'ideal %s' % (ideal,)
    def is_goal(p):
        return p in valid_edges

    def weight((x, y)):
        if costmap[y][x] < 127:
            # can't traverse
            return float('inf')

        if is_goal((x, y)):
            return abs(ideal[0] - x) + abs(ideal[1] - y)

        return 1

    center = MAP_SIZE_PIXELS / 2
    path = find_path(start=(x, y),
                     reached_goal=is_goal,
                     neighbors=grid_neighbors(costmap, jump_size=SPACING),
                     weight=weight,
                     heuristic=lambda v: manhattan(v, ideal))

    return path

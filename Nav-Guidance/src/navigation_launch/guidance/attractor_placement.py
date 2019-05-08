# fixed circle, sqrt(w/2) in diameter
# weight points across edge every SPACING points based on circle radius
# add arbitrary, linear weight to each point based on dist from ideal
# this breaks created by using circe radius

# also, probably use Dijkstra's since there's no meaningful heuristic we can use

import math

import cv2
import numpy as np

from a_star import find_path, grid_neighbors
from mapping import MAP_SIZE_PIXELS, pixel_to_byte, MAP_SIZE_METERS
from util import Vec2d

SPACING = 2
COSTMAP_DILATION_M = 0.8  # closest distance to obstacles pathfinding is allowed to get
CIRCLE_RADIUS = math.sqrt(2 * MAP_SIZE_PIXELS ** 2)


def edge_point():
    for x in xrange(MAP_SIZE_PIXELS / SPACING + 1):
        yield (x * SPACING, 0)
        yield (x * SPACING, MAP_SIZE_PIXELS - 1)
    for y in xrange(MAP_SIZE_PIXELS / SPACING + 1):
        yield (0, y * SPACING)
        yield (MAP_SIZE_PIXELS - 1, y * SPACING)


def navigable_edge_point((x, y), heading):
    angle = math.atan2(y - MAP_SIZE_PIXELS / 2.0,
                       x - MAP_SIZE_PIXELS / 2.0)
    return abs(math.degrees(angle) - heading) <= 90


def build_costmap(grid):
    # dilate obstacles
    dilation = int(float(COSTMAP_DILATION_M) / MAP_SIZE_METERS * MAP_SIZE_PIXELS)
    kernel = np.ones((dilation, dilation), np.uint8)
    img = cv2.bitwise_not(grid)
    img = cv2.dilate(img, kernel, iterations=1)
    img = cv2.bitwise_not(img)

    ret, img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)
    return img


def x_to_pixel(m):
    return int(m / MAP_SIZE_METERS * MAP_SIZE_PIXELS + MAP_SIZE_PIXELS / 2.0)


def y_to_pixel(m):
    return int(-m / MAP_SIZE_METERS * MAP_SIZE_PIXELS + MAP_SIZE_PIXELS / 2.0)


def generate_path(grid, heading, (x, y)):
    costmap = build_costmap(grid)
    valid_edges = set([e for e in edge_point() if navigable_edge_point(e, heading)])

    def is_goal(p):
        return p in valid_edges

    def weight((x, y)):
        if costmap[y][x] < 127:
            # can't traverse
            return float('inf')

        if is_goal((x, y)):
            ideal = Vec2d.from_point(math.cos(heading) * CIRCLE_RADIUS / 2 + MAP_SIZE_PIXELS / 2.0,
                                     math.sin(heading) * CIRCLE_RADIUS / 2 + MAP_SIZE_PIXELS / 2.0)
            return (ideal - Vec2d.from_point(x, y)).mag

        return 1

    x = int(round(x_to_pixel(x) / float(SPACING))) * SPACING
    y = int(round(y_to_pixel(y) / float(SPACING))) * SPACING

    center = MAP_SIZE_PIXELS / 2
    path = find_path(start=(x, y),
                     reached_goal=is_goal,
                     neighbors=grid_neighbors(costmap, jump_size=SPACING),
                     weight=weight)

    return costmap, path

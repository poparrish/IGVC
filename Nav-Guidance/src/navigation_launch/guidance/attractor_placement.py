# fixed circle, sqrt(w/2) in diameter
# weight points across edge every SPACING points based on circle radius
# add arbitrary, linear weight to each point based on dist from ideal
# this breaks created by using circe radius

# also, probably use Dijkstra's since there's no meaningful heuristic we can use

import math

import cv2
import numpy as np
from mapping import MAP_SIZE_PIXELS, pixel_to_byte, pixel_to_grid
from a_star import find_path, grid_neighbors, grid_traversable
from util import Vec2d

SPACING = 5
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
    img = np.array([pixel_to_byte(x) for x in grid.data], dtype=np.uint8)

    # # dilate obstacles
    img = np.reshape(img, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), order='F')
    ret, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    img = cv2.bitwise_not(img)

    kernel = np.ones((8, 8), np.uint8)
    img = cv2.dilate(img, kernel, iterations=1)
    img = cv2.bitwise_not(img)
    img = np.rot90(img)
    # cv2.imshow('test', img)
    # cv2.waitKey(3)

    return img


def generate_path(grid, heading):
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

    center = MAP_SIZE_PIXELS / 2
    path = find_path(start=(center, center),
                     reached_goal=is_goal,
                     neighbors=grid_neighbors(costmap, jump_size=SPACING),
                     weight=weight)

    return path

import math

import numpy as np

from util import Vec2d, clamp360

# TODO: Get this from camera_info?
WIDTH_PX = 420
HEIGHT_PX = 480 - 150

# TODO: Less bodge calculations, actually use trig to calculate projected image size
X_RES_MM = 3200.0 / WIDTH_PX
Y_RES_MM = 2200.0 / HEIGHT_PX
Y_OFFSET_MM = 920


def point_to_vector(p):
    [[x, y]] = p
    x = ((WIDTH_PX / 2) - x) * X_RES_MM
    y = (HEIGHT_PX - y) * Y_RES_MM - Y_OFFSET_MM
    v = Vec2d.from_point(x, y)
    return v.with_angle(v.angle - 90)


def contours_to_vectors(contours):
    return [[point_to_vector(p) for p in c] for c in contours]


def contour_area(vec_contours):
    if len(vec_contours) == 0:
        return 0

    f = vec_contours[0]
    min_x = max_x = f.x
    min_y = max_y = f.y

    for f in vec_contours:
        min_x = min(f.x, min_x)
        max_x = max(f.x, max_x)
        min_y = min(f.y, min_y)
        max_y = max(f.y, max_y)

    return (max_x - min_x) * (max_y - min_y)


def largest_contour(vec_contours):
    if len(vec_contours) == 0:
        return None

    return max(vec_contours, key=contour_area)


def calculate_line_angle(contours):
    contour = largest_contour(contours)

    if contour is None:
        return 0

    x = np.array([v.x for v in contour])
    y = np.array([v.y for v in contour])

    [slope, intercept] = np.polyfit(x, y, 1)
    return math.degrees(math.atan(slope))

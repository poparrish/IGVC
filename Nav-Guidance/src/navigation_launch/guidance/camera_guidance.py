import math

import numpy as np

from util import Vec2d

# TODO: Get this from camera_info?
WIDTH_PX = 640
HEIGHT_PX = 360
WIDTH_MM=5510
HEIGHT_MM=3556

# TODO: Less bodge calculations, actually use trig to calculate projected image size
X_RES_MM = 5510 / WIDTH_PX
Y_RES_MM = 3556 / HEIGHT_PX
Y_OFFSET_BOX_MM = HEIGHT_MM-((int(.88*360))*(HEIGHT_MM/HEIGHT_PX))
Y_OFFSET_CHASSIS_MM=965
Y_OFFSET_MM=Y_OFFSET_BOX_MM+Y_OFFSET_CHASSIS_MM
#Y_OFFSET_MM=0


def point_to_vector(p):
    [[x, y]] = p
    x = x*X_RES_MM
    y = (y* Y_RES_MM) + Y_OFFSET_MM
    #y*=-1
    x*=-1
    v = Vec2d.from_point(x, y)
    return v.with_angle(v.angle - 270)


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

# def calc_crab_away(contours):

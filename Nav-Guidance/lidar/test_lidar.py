from unittest import TestCase

from lidar import *
from util.vec2d import clamp

VALID_ANGLE = ANGLE_IGNORE_START - 10
VALID_DIST = MIN_DIST_CM + 1


def scan(angle, dist):
    return None, angle, dist


class LidarTest(TestCase):
    def test_filters_angles(self):
        res = vectorize_scan([scan(ANGLE_IGNORE_START, VALID_DIST),
                              scan(ANGLE_IGNORE_START + 1, VALID_DIST),
                              scan(ANGLE_IGNORE_END, VALID_DIST),
                              scan(VALID_ANGLE, VALID_DIST)])

        self.assertEquals([r.angle for r in res], [clamp(VALID_ANGLE - 180)])

    def test_filters_distances(self):
        res = vectorize_scan([scan(0, MIN_DIST_CM - 1),
                              scan(10, VALID_DIST),
                              scan(20, MAX_DIST_CM + 1)])

        self.assertEquals([r.mag for r in res], [VALID_DIST])

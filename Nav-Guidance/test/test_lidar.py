from nose.tools import assert_equal

from lidar.lidar_node import *

VALID_ANGLE = ANGLE_IGNORE_START - 10
VALID_DIST = MIN_DIST_MM + 1


def scan(angle, dist):
    return None, angle + LIDAR_OFFSET_ANGLE, dist


def test_filters_angles():
    res = convert_scan_to_vectors([scan(ANGLE_IGNORE_START, VALID_DIST),
                                   scan(ANGLE_IGNORE_START + 1, VALID_DIST),
                                   scan(ANGLE_IGNORE_END, VALID_DIST),
                                   scan(VALID_ANGLE, VALID_DIST)])

    assert_equal([r.angle for r in res], [VALID_ANGLE])


def test_filters_distances():
    res = convert_scan_to_vectors([scan(0, MIN_DIST_MM - 1),
                                   scan(10, VALID_DIST),
                                   scan(20, MAX_DIST_MM + 1)])

    assert_equal([r.mag for r in res], [VALID_DIST])

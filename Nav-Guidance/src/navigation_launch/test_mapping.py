from nose.tools import assert_equals

from parameterized import parameterized

from mapping import Map
from util import Vec2d


@parameterized([
    (50, False),
    (35, True),
    (0, True),
    (360 - 35, True),
    (360 - 50, False),
])
def test_in_range_camera(angle, expected):
    m = Map('camera_test', detection_angle_degrees=70)
    assert_equals(m.in_range(Vec2d(angle, 0)), expected)


@parameterized([
    (145, False),
    (135, True),
    (90, True),
    (45, True),
    (0, True),
    (360 - 45, True),
    (360 - 90, True),
    (360 - 135, True),
    (360 - 145, False),
])
def test_in_range_lidar(angle, expected):
    m = Map('lidar_test', detection_angle_degrees=270)
    assert_equals(m.in_range(Vec2d(angle, 0)), expected)


@parameterized([
    (35, 180),
    (35 / 2.0, 90),
    (0, 0),
    (360 - 35 / 2.0, 270),
    (360 - 35, 180),
])
def test_skew_camera(angle, expected):
    m = Map('camera_test', detection_angle_degrees=70)
    assert_equals(m.skew(angle), expected)


@parameterized([
    (135, 180),
    (67.5, 90),
    (0, 0),
    (360 - 135 / 2.0, 270),
    (360 - 135, 180),
])
def test_skew_lidar(angle, expected):
    m = Map('lidar_test', detection_angle_degrees=270)
    assert_equals(m.skew(angle), expected)

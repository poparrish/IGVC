import math
from nose.tools import assert_almost_equal, assert_equal

from nose_parameterized import parameterized

from util import Vec2d, to180, to360


def test_x():
    v = Vec2d(30, 100)
    assert_almost_equal(v.x, 86.6025, places=3)


def test_y():
    v = Vec2d(30, 100)
    assert_almost_equal(v.y, 50, places=3)


def test_add():
    # Q1
    v = Vec2d(0, 100) + Vec2d(90, 100)
    assert_almost_equal(v.angle, 45, places=3)
    assert_almost_equal(v.mag, 141.4213, places=3)

    # Q2
    v = Vec2d(-90, 100) + Vec2d(0, 100)
    assert_almost_equal(v.angle, 315, places=3)
    assert_almost_equal(v.mag, 141.4213, places=3)

    # Q3
    v = Vec2d(-90, 100) + Vec2d(-180, 100)
    assert_almost_equal(v.angle, 225, places=3)
    assert_almost_equal(v.mag, 141.4213, places=3)

    # Q4
    v = Vec2d(90, 100) + Vec2d(-180, 100)
    assert_almost_equal(v.angle, 135, places=3)
    assert_almost_equal(v.mag, 141.4213, places=3)

    # Div by 0
    v = Vec2d(0, 100) + Vec2d(0, 100)
    assert_almost_equal(v.angle, 0, places=3)
    assert_almost_equal(v.mag, 200, places=3)


@parameterized([
    (0, 0),
    (90, 90),
    (180, 180),
    (270, -90),
    (360, 0),
    (450, 90),
    (-90, -90),
    (-180, 180),
    (-270, 90),
    (-360, 0),
    (-450, -90)
])
def test_to180(angle, expected):
    assert_equal(to180(angle), expected)


@parameterized([
    (0, 0),
    (90, 90),
    (180, 180),
    (270, 270),
    (360, 0),
    (450, 90),
    (-90, 270),
    (-180, 180),
    (-270, 90),
    (-360, 0),
    (-450, 270)
])
def test_to360(angle, expected):
    assert_equal(to360(angle), expected)


@parameterized([
    # y
    (0, 100, Vec2d(90, 100)),
    (0, -100, Vec2d(270, 100)),
    # x
    (100, 0, Vec2d(0, 100)),
    (-100, 0, Vec2d(180, 100)),
    # both
    (100, 100, Vec2d(45, math.sqrt(20000))),
    (100, -100, Vec2d(315, math.sqrt(20000))),
    (-100, 100, Vec2d(135, math.sqrt(20000))),
    (-100, -100, Vec2d(225, math.sqrt(20000)))
])
def test_from_point(x, y, expected):
    actual = Vec2d.from_point(x, y)
    assert_equal((actual.angle, actual.mag), (expected.angle, expected.mag))
    (-100, -100, Vec2d(0, 100)),

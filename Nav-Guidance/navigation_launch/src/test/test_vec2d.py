from nose.tools import assert_almost_equal

from util import Vec2d


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

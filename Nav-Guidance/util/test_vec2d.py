from unittest import TestCase

from util import Vec2d


class TestVec2d(TestCase):
    def test_x(self):
        v = Vec2d(30, 100)
        self.assertAlmostEquals(v.x, 86.6025, places=3)

    def test_y(self):
        v = Vec2d(30, 100)
        self.assertAlmostEquals(v.y, 50, places=3)

    def test_add(self):
        # Q1
        v = Vec2d(0, 100) + Vec2d(90, 100)
        self.assertAlmostEquals(v.angle, 45, places=3)
        self.assertAlmostEquals(v.mag, 141.4213, places=3)

        # Q2
        v = Vec2d(-90, 100) + Vec2d(0, 100)
        self.assertAlmostEquals(v.angle, 315, places=3)
        self.assertAlmostEquals(v.mag, 141.4213, places=3)

        # Q3
        v = Vec2d(-90, 100) + Vec2d(-180, 100)
        self.assertAlmostEquals(v.angle, 225, places=3)
        self.assertAlmostEquals(v.mag, 141.4213, places=3)

        # Q4
        v = Vec2d(90, 100) + Vec2d(-180, 100)
        self.assertAlmostEquals(v.angle, 135, places=3)
        self.assertAlmostEquals(v.mag, 141.4213, places=3)

        # Div by 0
        v = Vec2d(0, 100) + Vec2d(0, 100)
        self.assertAlmostEquals(v.angle, 0, places=3)
        self.assertAlmostEquals(v.mag, 200, places=3)

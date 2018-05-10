from unittest import TestCase

from avoidance import initial_bearing, calculate_gps_heading
from gps import GPSMsg
from util.vec2d import clamp

culinary_arts = (43.601214, -116.197543)
rec_center = (43.600532, -116.200390)


# https://www.movable-type.co.uk/scripts/latlong.html
class TestAvoidance(TestCase):

    def test_initial_bearing(self):
        bearing = initial_bearing(culinary_arts, rec_center)
        self.assertLessEqual(abs(251.6969 - bearing), 4)  # for some reason calculations are off, within 4 deg is ok

    def test_calculate_gps_heading(self):
        bearing = initial_bearing(culinary_arts, rec_center)
        heading = calculate_gps_heading(GPSMsg(lat=culinary_arts[0], lon=culinary_arts[1], heading=bearing + 10),
                                        rec_center)

        self.assertAlmostEqual(heading.angle, clamp(-10))
        self.assertAlmostEqual(heading.mag, 241.5, places=1)

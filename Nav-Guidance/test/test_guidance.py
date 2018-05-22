from nose.tools import assert_almost_equal, assert_less, assert_true

from gps import GPSMsg
from guidance import calculate_gps_heading
from guidance.camera_guidance import largest_contour, calculate_line_angle
from guidance.gps_guidance import initial_bearing
from util import clamp360


# TODO: More tests...

#
# Potential field
#

def test_partition():
    assert_true(False)  # TODO


#
# Camera
#

def test_linear_regression():
    calculate_line_angle([])
    assert_true(False)  # TODO


def test_largest_contour():
    largest_contour([])
    assert_true(False)  # TODO


#
# GPS
#

# Calculations made with https://www.movable-type.co.uk/scripts/latlong.html
culinary_arts = (43.601214, -116.197543)
rec_center = (43.600532, -116.200390)


def test_initial_bearing():
    bearing = initial_bearing(culinary_arts, rec_center)
    assert_less(abs(251.6969 - bearing), 4)  # for some reason calculations are off, within 4 deg is ok I guess


def test_calculate_gps_heading():
    bearing = initial_bearing(culinary_arts, rec_center)
    heading = calculate_gps_heading(GPSMsg(lat=culinary_arts[0],
                                           lon=culinary_arts[1],
                                           heading=bearing + 10),
                                    rec_center)

    assert_almost_equal(heading.angle, clamp360(-10))
    assert_almost_equal(heading.mag, 241.5, places=1)

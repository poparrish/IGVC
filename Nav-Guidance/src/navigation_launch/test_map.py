from breezyslam.sensors import Laser
from nose_parameterized import parameterized

from map import Map
from util import Vec2d


class TestLaser(Laser):
    def __init__(self, scan_size, detection_angle_degrees, scan_rate_hz=5.0, distance_no_detection_mm=5000):
        super(TestLaser, self).__init__(scan_size, scan_rate_hz, detection_angle_degrees, distance_no_detection_mm)


@parameterized([
    (TestLaser(scan_size=180, detection_angle_degrees=90),
     [Vec2d(0, 1.0),
      Vec2d(5, 0.5)],
     [Vec2d(0, 1.0),
      Vec2d(0, 0.95),
      Vec2d(0, 0.90),
      Vec2d(0, 0.85),
      Vec2d(0, 0.80),
      Vec2d(0, 0.75),
      Vec2d(0, 0.70),
      Vec2d(0, 0.65),
      Vec2d(0, 0.60),
      Vec2d(0, 0.55),
      Vec2d(5, 0.50)]
     )
])
def test_normalize_scan():
    Map(size_px=10, size_meters=)
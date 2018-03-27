from breezyslam.sensors import Laser


class A1M8(Laser):
    """
    SLAM model for the RPLidar A1M8
    """

    def __init__(self, detection_margin=0, offset_mm=0):
        Laser.__init__(self,
                       scan_size=144,
                       scan_rate_hz=10,
                       detection_angle_degrees=360,
                       distance_no_detection_mm=5,
                       detection_margin=detection_margin,
                       offset_mm=offset_mm)

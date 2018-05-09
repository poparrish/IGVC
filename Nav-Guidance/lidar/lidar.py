import rospy
from breezyslam.algorithms import RMHC_SLAM
from rplidar import RPLidar
import cv2
import numpy as np

from A1M8 import *
from pltslamshow import SlamShow

LIDAR_NODE = 'LIDAR'

MAP_SIZE_PIXELS = 50
MAP_SIZE_METERS = 10


def resize_scan(scan, scan_size):
    """
    The RPLidar A1M8 does not return a consistent number of scans every cycle, which is a
    requirement for BreezySLAM.
    This function shrinks/grows the scan as needed using "nearest-neighbor" approximation
    """
    conv = len(scan) / float(scan_size)

    # Extract the third element of (quality, angle, scan) tuples
    return [scan[int(i * conv)][2] for i in range(scan_size)]


def lidar_start(device):
    lidar = RPLidar(device)
    slam = RMHC_SLAM(A1M8(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    # display = SlamShow(MAP_SIZE_PIXELS, MAP_SIZE_METERS * 1000 / MAP_SIZE_PIXELS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    # map = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    maxx = MAP_SIZE_METERS * 1000 / MAP_SIZE_PIXELS
    maxy = MAP_SIZE_METERS * 1000 / MAP_SIZE_PIXELS
    midx = maxx/2
    midy = maxy/2

    while not rospy.is_shutdown():
        for i, scan in enumerate(lidar.iter_scans()):
            lidar_map = np.zeros([maxx, maxy, 3], np.uint8)
            print((maxx, maxy))
            for s in scan:
                theta = np.radians(s[1])
                # print(theta)
                dist = s[2]
                deltax = int(np.cos(theta)*dist)
                deltay = int(np.sin(theta)*dist)
                # print((theta, deltax, deltay))
                cv2.circle(lidar_map,(midx+deltax, midy + deltay), 1, (0,255,255))
                # print s
            cv2.imshow('test', lidar_map)
            # Update SLAM with the latest Lidar scan
            # slam.update(resize_scan(scan, slam.laser.scan_size))

            # Get current robot position
            # x, y, theta = slam.getpos()
            #print(x, y, theta)

            # Get current map bytes as grayscale
            # slam.getmap(map)

            # display.displayMap(map)
            # display.setPose(x, y, theta)

            # if not display.refresh():
            #     exit(0)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
        rate.sleep()
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    lidar_start('/dev/ttyUSB0')

import pickle

import rospy
from rplidar import RPLidar, MAX_MOTOR_PWM
from std_msgs.msg import String

from util.vec2d import Vec2d

LIDAR_NODE = 'LIDAR'

MIN_DIST_CM = 500
MAX_DIST_CM = 5000

ANGLE_IGNORE_START = 150
ANGLE_IGNORE_END = 210


def create_vector((quality, angle, dist)):
    valid_angle = not ANGLE_IGNORE_START <= angle <= ANGLE_IGNORE_END
    valid_distance = MIN_DIST_CM < dist < MAX_DIST_CM
    if valid_angle and valid_distance:
        return Vec2d(angle + 180, dist)


def vectorize_scan(scan):
    return [v for v in map(create_vector, scan) if v is not None]


def lidar_start(device):
    rospy.init_node(LIDAR_NODE)
    pub = rospy.Publisher(LIDAR_NODE, String, queue_size=10)

    lidar = RPLidar(device)
    lidar.set_pwm(MAX_MOTOR_PWM)  # set to full speed

    while not rospy.is_shutdown():
        for i, scan in enumerate(lidar.iter_scans()):
            scan = vectorize_scan(scan)
            pub.publish(pickle.dumps(scan))


if __name__ == '__main__':
    lidar_start('/dev/ttyUSB0')

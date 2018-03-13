import math
import pickle

import rospy
from std_msgs.msg import String

LIDAR_NODE = 'LIDAR'
LIDAR_REFRESH_RATE = 50
LIDAR_DEPRESSION = 10 * math.pi / 180  # angle of depression


class Vec2:
    def __init__(self, magnitude, direction):
        self.magnitude = magnitude
        self.direction = direction

    def __str__(self):
        return vars(self)


def update_angle():
    # TODO: Sweep
    return 0


def get_angle():
    # TODO: Get servo position
    return 0


def get_distance():
    # TODO: Poll lidar
    return 0


def lidar_start():
    pub = rospy.Publisher(LIDAR_NODE, String, queue_size=LIDAR_REFRESH_RATE)
    rospy.init_node(LIDAR_NODE, anonymous=True)
    rate = rospy.Rate(LIDAR_REFRESH_RATE)

    while not rospy.is_shutdown():
        dist = get_distance()
        angle = get_angle()

        # calculate distance TODO correct for height?
        dist = math.cos(LIDAR_DEPRESSION) * dist

        data = Vec2(magnitude=dist, direction=angle)
        pub.publish(pickle.dumps(data))
        rate.sleep()


if __name__ == '__main__':
    lidar_start()

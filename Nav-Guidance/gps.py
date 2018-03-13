import pickle

import rospy
from std_msgs.msg import String

GPS_NODE = 'GPS'
GPS_REFRESH_RATE = 10


class GPSPos:
    def __init__(self, lat, lon, direction):
        self.lat = lat
        self.lon = lon
        self.direction = direction

    def __str__(self):
        return vars(self)


def get_location():
    # TODO get gps location
    return GPSPos(lat=0, lon=0, direction=0)


def gps_start():
    pub = rospy.Publisher(GPS_NODE, String, queue_size=GPS_REFRESH_RATE)
    rospy.init_node(GPS_NODE, anonymous=True)
    rate = rospy.Rate(GPS_REFRESH_RATE)

    while not rospy.is_shutdown():
        pos = get_location()
        print(vars(pos))
        pub.publish(pickle.dumps(pos))
        rate.sleep()


if __name__ == '__main__':
    gps_start()

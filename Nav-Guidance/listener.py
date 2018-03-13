from gps import *
from lidar import *


def gps_updated(data):
    coords = pickle.loads(data.data)
    rospy.loginfo("gps %s" % vars(coords))


def lidar_updated(data):
    vec = pickle.loads(data.data)
    rospy.loginfo("lidar %s" % vars(vec))


def nav_start():
    rospy.init_node('nav', anonymous=True)

    rospy.Subscriber(GPS_NODE, String, gps_updated)
    rospy.Subscriber(LIDAR_NODE, String, lidar_updated)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # TODO: Update
        rate.sleep()


if __name__ == '__main__':
    nav_start()

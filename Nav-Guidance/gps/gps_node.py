import pickle
import time

import rospy
from pymavlink import mavutil
from std_msgs.msg import String

from gps_msg import GPSMsg

GPS_NODE = 'GPS'
GPS_REFRESH_RATE = 10


def get_location(mav):
    if 'GLOBAL_POSITION_INT' in mav.messages:
        msg = mav.messages['GLOBAL_POSITION_INT']
        return GPSMsg(lat=msg.lat / 10.0 ** 7,
                      lon=msg.lon / 10.0 ** 7,
                      heading=msg.hdg / 100.0)
    return None


def mavlink_init(device):
    mav = mavutil.mavlink_connection(device, baud=115200, autoreconnect=True)

    rospy.logdebug('Waiting for HEARTBEAT')
    mav.wait_heartbeat()

    return mav


def gps_start(device):
    pub = rospy.Publisher(GPS_NODE, String, queue_size=GPS_REFRESH_RATE * 10)
    rospy.init_node(GPS_NODE)

    mav = mavlink_init(device)
    pevent = mavutil.periodic_event(GPS_REFRESH_RATE)

    while not rospy.is_shutdown():
        mav.recv_msg()

        if pevent.trigger():
            loc = get_location(mav)
            if loc is not None:
                rospy.loginfo(vars(loc))
                pub.publish(pickle.dumps(loc))

        time.sleep(0.01)


if __name__ == '__main__':
    gps_start('/dev/ttyACM0')

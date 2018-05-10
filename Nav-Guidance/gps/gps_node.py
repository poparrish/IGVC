import pickle
import time

import rospy
from pymavlink import mavutil
from pymavlink.dialects.v20.common import GPS_FIX_TYPE_3D_FIX
from std_msgs.msg import String

from gps_msg import GPSMsg

GPS_NODE = 'GPS'
GPS_REFRESH_RATE = 10


def get_location(mav):
    pos = mav.messages.get('GLOBAL_POSITION_INT', None)
    raw = mav.messages.get('GPS_RAW_INT', None)

    if pos is not None and raw is not None:
        return GPSMsg(lat=pos.lat / 10.0 ** 7,
                      lon=pos.lon / 10.0 ** 7,
                      heading=pos.hdg / 100.0,
                      fixed=raw.fix_type >= GPS_FIX_TYPE_3D_FIX)
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

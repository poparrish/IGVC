#!/usr/bin/env python
import pickle
import time

import rospy
from pymavlink import mavutil
from pymavlink.dialects.v20.common import GPS_FIX_TYPE_3D_FIX
from std_msgs.msg import String

GPS_NODE = 'GPS'
GPS_REFRESH_RATE = 10


def get_location(mav):
    pos = mav.messages.get('GLOBAL_POSITION_INT', None)
    raw = mav.messages.get('GPS_RAW_INT', None)

    if pos is not None and raw is not None:
        return {'lat': pos.lat / 10.0 ** 7,
                'lon': pos.lon / 10.0 ** 7,
                'heading': pos.hdg / 100.0,
                'fixed': raw.fix_type >= GPS_FIX_TYPE_3D_FIX}
    return None


def init_mavlink(device):
    mav = mavutil.mavlink_connection(device, baud=115200, autoreconnect=True)

    rospy.logdebug('Waiting for HEARTBEAT')
    mav.wait_heartbeat()

    return mav


def start_gps(device):
    pub = rospy.Publisher(GPS_NODE, String, queue_size=GPS_REFRESH_RATE * 10)
    rospy.init_node(GPS_NODE)

    mav = init_mavlink(device)
    pevent = mavutil.periodic_event(GPS_REFRESH_RATE)

    while not rospy.is_shutdown():
        mav.recv_msg()

        if pevent.trigger():
            loc = get_location(mav)
            if loc is not None:
                print(loc['lat'], loc['lon'])
                rospy.loginfo('Publishing gps %s' % loc)
                pub.publish(pickle.dumps(loc))

        #why this and not ros.wait()
        time.sleep(0.01)


if __name__ == '__main__':
    start_gps('/dev/pixhawk')

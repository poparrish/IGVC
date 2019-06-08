#!/usr/bin/env python
import pickle
import time

import rospy
from pymavlink import mavutil, mavextra
from pymavlink.dialects.v20.common import GPS_FIX_TYPE_3D_FIX
from std_msgs.msg import String, Int32
import numpy as np

import topics

GPS_NODE = 'GPS'
GPS_REFRESH_RATE = 30
GPS_BIAS = 0  # 160-90-30#58

xOffset = 0
yOffset = 0
zOffset = 0

Pitch = 0
Roll = 0
Yaw = 0

# heading = 0


def get_location(mav):
    global heading
    pos = mav.messages.get('GLOBAL_POSITION_INT', None)
    raw = mav.messages.get('GPS_RAW_INT', None)
    # print "RAW: ",pos

    attitude=mav.messages.get('ATTITUDE',None)

    if pos is not None and raw is not None and attitude is not None:
        return {'lat': pos.lat / 10.0 ** 7,
                'lon': pos.lon / 10.0 ** 7,
                'satellites': raw.satellites_visible,
                'gps_heading': int(pos.hdg/100),
                'heading': 360 - int(np.rad2deg(attitude.yaw)),  # just whole #'s
                'fixed': raw.fix_type >= GPS_FIX_TYPE_3D_FIX,
                'pitch': int(np.rad2deg(attitude.pitch)),
                'roll': int(np.rad2deg(attitude.roll))}
    return None


def init_mavlink(device):
    mav = mavutil.mavlink_connection(device, baud=115200, autoreconnect=True)

    rospy.logdebug('Waiting for HEARTBEAT')
    mav.wait_heartbeat()

    return mav


# def new_heading(data):
#     global heading
#     heading = data.data


def start_gps(device):
    rospy.init_node(GPS_NODE)

    pub = rospy.Publisher(topics.GPS, String, queue_size=GPS_REFRESH_RATE * 10)
    #rospy.Subscriber('heading', Int32, new_heading)
    heading_pub=rospy.Publisher(topics.ORIENTATION,Int32,queue_size=0)
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
                heading_pub.publish(loc['heading'])

        # why this and not ros.wait()
        time.sleep(0.01)


if __name__ == '__main__':
    start_gps('/dev/pixhawk')

#!/usr/bin/env python
import pickle

import math
import numpy as np
import rospy
from cameras import CAMERA_NODE
from util import rx_subscribe
from rx import Observable
from std_msgs.msg import String
from lidar import LIDAR_TOPIC
from lidar import start_lidar_noROS
from line_map_nav import LINE_MAP_HZ, LINE_MAP_NAV_NODE
from rx import Observable

import matplotlib.pyplot as plt
import matplotlib.animation as animation


from guidance import contours_to_vectors


LINE_MAP_GUIDANCE_NODE='LINE_MAP_GUIDANCE_NODE'

def vector_to_point(vector):
    return vector.x, vector.y

def vectors_to_points(vector_contours):
    return [[vector_to_point(v) for v in c]for c in vector_contours]

def build_point_cloud(vectors):
    """this should just report the closest object """
    return vectors_to_points(vectors)



def flatten_contours(pointCloud):
    """merges contours, and makes them mutable list items"""
    cloud = []
    contour_count=0
    for contour in pointCloud:
        contour_count+=1
        for point in contour:
            cloud.append(point)
    # print "CONTOUR_COUNT: ",contour_count
    # print "CLOUD: ",cloud
    return cloud

def flatten_axes(pointCloud):
    """separates x & y for plotting, should not be any contours. just [(),()]"""
    xaxis = []
    yaxis = []
    for point in pointCloud:
        xaxis.append(int(point.x))
        yaxis.append(int(point.y))
    return xaxis,yaxis


def update_raw_map((msg)):
    """figures out what we need to do based on the current state and map"""
    print "updating_raw_map"
    camera = msg['camera']
    lidar = msg['lidar']

    #lidarPointCloud = build_point_cloud(lidar)
    camera_cloud = flatten_contours(camera)#this returns lists, not Vec2d object input

    #convert to depth map
    #get_depth_map(lidar, camera_cloud) #this returns lists not Vec2d Object input
    global lx,ly
    lx,ly = flatten_axes(lidar)
    global cx, cy
    cx,cy = flatten_axes(camera_cloud)
    # plt.show()
    # plt.draw()

    # x = lx+cx
    # y = ly+cy
    #
    # update_plot(lx,ly,cx,cy)

def update_plot(lx,ly,cx,cy):

    for i in range(len(lx)):
        lidar_plot.set_xdata(lx[0:i])
    for i in range(len(ly)):
        lidar_plot.set_ydata(ly[0:i])
    for i in range(len(cx)):
        camera_plot.set_xdata(cx[0:i])
    for i in range(len(cy)):
        camera_plot.set_ydata(cy[0:i])


    print"updated?"

def main():

    rospy.init_node(LINE_MAP_GUIDANCE_NODE)


    # we randomly seem to get garbage messages that are only partially unpickled
    # ignore them until we can figure out what's going on
    def valid_message(msg):
        return not isinstance(msg['camera'], basestring)

    nav = rx_subscribe(LINE_MAP_NAV_NODE).filter(valid_message)


    # update controls whenever nav or state emits
    Observable.combine_latest(nav, lambda n: (n)) \
        .subscribe(update_raw_map)


    rate = rospy.Rate(LINE_MAP_HZ)


    while not rospy.is_shutdown():
        global cx, cy, lx, ly
        nplx = np.array(lx)
        nply = np.array(ly)
        npcx = np.array(cx)
        npcy = np.array(cy)

        i = len(nplx)
        lidar_plot.set_xdata(nply[0:i])
        i = len(nply)
        lidar_plot.set_ydata(nplx[0:i])
        i = len(npcx)
        camera_plot.set_xdata(npcy[0:i])
        i = len(npcy)
        camera_plot.set_ydata(npcx[0:i])
        plt.pause(.1)

        rate.sleep()

if __name__ == "__main__":
    lx=[]
    ly=[]
    cx=[]
    cy=[]
    plt.ion()
    plt.axis([-4000, 4000, 4000, -4000])
    camera_plot= plt.plot([], [], 'ro')[0]
    lidar_plot = plt.plot([],[],'bo')[0]
    origin_plot = plt.plot(0,0,'yo')
    plt.show()
    plt.draw()
    main()
#!/usr/bin/env python
from enum import Enum
import pickle
import cv2
# temp
import numpy as np
import rospy
from std_msgs.msg import String, Int16

import topics
from camera_info import CameraInfo
from camera_msg import CameraMsg
import math
import pprint
from util import rx_subscribe, Vec2d
from rx import Observable
from guidance import contours_to_vectors




cam_name = '/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_2B2150DE-video-index0'
cam_name = 1


def callback(x):
    #the cv2.createTrackbar() requires callback param
    pass

def process_image(img, camera_info):
    #median blur
    medianRadius = cv2.getTrackbarPos('medianRadius', 'img_medianBlur')
    img_medianBlur = blur(src = img,type = BlurType.Median_Filter, radius = medianRadius)
    cv2.namedWindow('img_medianBlur',0)
    cv2.resizeWindow('img_medianBlur', 640, 480)
    #cv2.imshow('img_medianBlur',img_medianBlur)


    #HSV filter
    ilowH = cv2.getTrackbarPos('lowH','img_HSV')
    ihighH = cv2.getTrackbarPos('highH','img_HSV')
    ilowS = cv2.getTrackbarPos('lowS','img_HSV')
    ihighS = cv2.getTrackbarPos('highS','img_HSV')
    ilowV = cv2.getTrackbarPos('lowV','img_HSV')
    ihighV = cv2.getTrackbarPos('highV','img_HSV')
    hue_threshold=[ilowH,ihighH]
    sat_threshold=[ilowS,ihighS]
    val_threshold=[ilowV,ihighV]
    img_HSV = hsv_threshold(input=img_medianBlur,hue=hue_threshold,sat=sat_threshold,val=val_threshold)
    #img_HSV = rgb_threshold(img_medianBlur,hue_threshold,sat_threshold,val_threshold)

    cv2.imshow('img_HSV',img_HSV)



    #gaussian blur
    gaussianRadius = cv2.getTrackbarPos('gaussianRadius', 'img_gaussianBlur')
    img_gaussianBlur = blur(src=img_HSV,type=BlurType.Gaussian_Blur,radius=gaussianRadius)
    #cv2.imshow('img_gaussianBlur',img_gaussianBlur)

    #birds eye view
    img_displayBirdsEye = camera_info.convertToFlat(img_gaussianBlur)
    #cv2.imshow("birdsEye", img_displayBirdsEye)

    #find contours
    contoursMinArea = cv2.getTrackbarPos('contoursMinArea', 'img_displayFilteredContours')
    contoursMinPerimeter = cv2.getTrackbarPos('contoursMinPerimeter', 'img_displayFilteredContours')
    contoursMinWidth = cv2.getTrackbarPos('contoursMinWidth', 'img_displayFilteredContours')
    contoursMaxWidth = cv2.getTrackbarPos('contoursMaxWidth', 'img_displayFilteredContours')
    contoursMinHeight = cv2.getTrackbarPos('contoursMinHeight', 'img_displayFilteredContours')
    contoursMaxHeight = cv2.getTrackbarPos('contoursMaxHeight', 'img_displayFilteredContours')
    contoursSolidity = [21, 100]
    contoursSolidityMin = cv2.getTrackbarPos('contoursSolidityMin', 'img_displayFilteredContours')
    contoursSolidityMax = cv2.getTrackbarPos('contoursSolidityMax', 'img_displayFilteredContours')
    contoursMaxVertices = cv2.getTrackbarPos('contoursMaxVertices', 'img_displayFilteredContours')
    contoursMinVertices = cv2.getTrackbarPos('contoursMinVertices', 'img_displayFilteredContours')
    contoursMinRatio = cv2.getTrackbarPos('contoursMinRatio', 'img_displayFilteredContours')
    contoursMaxRatio = cv2.getTrackbarPos('contoursMaxRatio', 'img_displayFilteredContours')
    img_rawContours = find_contours(input=img_displayBirdsEye,external_only=False)
    img_displayRawContours=np.ones_like(img_displayBirdsEye)#Return an array of ones with the same shape and type as a given array.
    cv2.drawContours(img_displayRawContours, img_rawContours, -1, (255, 255, 255), thickness=1) #-1 thickness makes them solid
    #cv2.imshow('img_displayRawContours',img_displayRawContours)

    #filter contours
    img_filteredContours = filter_contours(input_contours=img_rawContours,min_area=contoursMinArea,min_perimeter=contoursMinPerimeter,
                                           min_width=contoursMinWidth,max_width=contoursMaxWidth,min_height=contoursMinHeight,
                                           max_height=contoursMaxHeight,solidity=[contoursSolidityMin,contoursSolidityMax],
                                           max_vertex_count=contoursMaxVertices,min_vertex_count=contoursMinVertices,min_ratio=contoursMinRatio,
                                           max_ratio=contoursMaxRatio)
    img_displayFilteredContours = np.ones_like(img)  # Return an array of ones with the same shape and type as a given array.
    cv2.drawContours(img_displayFilteredContours, img_filteredContours, -1, (255, 255, 255), thickness=-1)
    cv2.imshow('img_displayFilteredContours', img_displayFilteredContours)

    return img_displayBirdsEye, img_filteredContours

def filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,min_height, max_height, solidity, max_vertex_count, min_vertex_count,min_ratio, max_ratio):
    output = []
    for contour in input_contours:
        x, y, w, h = cv2.boundingRect(contour)
        if (w < min_width or w > max_width):
            continue
        if (h < min_height or h > max_height):
            continue
        area = cv2.contourArea(contour)
        if (area < min_area):
            continue
        if (cv2.arcLength(contour, True) < min_perimeter):
            continue
        hull = cv2.convexHull(contour)
        solid = 100 * area / cv2.contourArea(hull)
        if (solid < solidity[0] or solid > solidity[1]):
            continue
        if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
            continue
        ratio = (float)(w) / h
        if (ratio < min_ratio or ratio > max_ratio):
            continue
        output.append(contour)
    return output

def find_contours(input,external_only):
    if (external_only):
        mode = cv2.RETR_EXTERNAL
    else:
        mode = cv2.RETR_LIST
    method = cv2.CHAIN_APPROX_NONE
    im2, contours, hierarchy = cv2.findContours(input, mode=mode, method=method)
    return contours

def hsv_threshold(input, hue, sat, val):
    out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    return cv2.inRange(out, (hue[0], sat[0], val[0]), (hue[1], sat[1], val[1]))

def rgb_threshold(input, r,g,b):
    out = cv2.cvtColor(input, cv2.COLOR_BGR2RGB)
    return cv2.inRange(out, (r[0], g[0], b[0]), (r[1], g[1], b[1]))

def blur(src, type, radius):
    if (type is BlurType.Box_Blur):
        ksize = int(2 * round(radius) + 1)
        return cv2.blur(src, (ksize, ksize))
    elif (type is BlurType.Gaussian_Blur):
        ksize = int(6 * round(radius) + 1)
        return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
    elif (type is BlurType.Median_Filter):
        ksize = int(2 * round(radius) + 1)
        return cv2.medianBlur(src, ksize)
    else:
        return cv2.bilateralFilter(src, -1, round(radius), round(radius))

def convert_to_cartesian(WIDTH,HEIGHT, contours):
    """to merge everything with the lidar we need to set our x origin to the center of the frame and our y origin
    to where the lidar is. this happens to be at the same height in pixels as the blackout box we configure in the
    CameraInfo class.rectangle_height.
    For some fucked up reason the contours that CV outputs has an extra list layer around the individual points...
    the point_ is just going through that layer. so don't let it confuse
    To avoid looping a second time we also have the option to remove 'thin' the contours by removing
    n number of points along the perimiter"""
    for contour in contours:
        for point_ in contour:

            for point in point_:#this is the layer in each contour that has the points
                #shift x to center

                if point[0] > WIDTH/2:
                    point[0] = point[0]-WIDTH/2
                elif point[0] < WIDTH / 2:
                    point[0] = WIDTH / 2 - point[0]
                    point[0] *= -1
                else:
                    point[0] = 0

                #shift y to center
                if point[1] > HEIGHT / 2:
                    point[1] = point[1] - HEIGHT / 2
                    point[1] *= -1
                elif point[1] < HEIGHT / 2:
                    point[1] = HEIGHT / 2 - point[1]
                else:
                    point[1] = 0
        # print contour
    return contours

def contour_slope(contour):
    if contour is None:
        return 0

    x = np.array([v.x for v in contour])
    y = np.array([v.y for v in contour])

    [slope, intercept] = np.polyfit(x, y, 1)
    return math.degrees(math.atan(slope))

def closest_contour_slope(contours):
    i = 0#track which contour
    results = []
    for contour in contours:#find closest point per contour
        closest = 10000000#just a large starting #
        for vec in contour:
            if closest > vec.mag:
                closest = vec.mag
        results.append([i,closest])
        i+=1

    closest_contour = []
    for result in results:
        closest = 10000000
        if closest > result[1]:
            closest = result[1]
            closest_contour = contours[result[0]]

    if len(closest_contour) == 0:
        return 0
    else:
        return contour_slope(closest_contour)

def closest_contour(contours):
    i = 0#track which contour
    results = []
    for contour in contours:#find closest point per contour
        closest = 10000000#just a large starting #
        for vec in contour:
            if closest > vec.mag:
                closest = vec.mag
        results.append([i,closest,vec.contour_group])
        i+=1

    closest_contour = []
    closest = 10000000
    for result in results:
        if closest > result[1]:
            closest = result[1]
            closest_contour = contours[result[0]]
    if len(closest_contour) == 0:
        return 0
    else:
        return closest_contour

def flatten_contours(pointCloud):
    """merges contours, but preserves grouping. group #'s are arbitrary"""
    cloud = []
    contour_count=0
    for contour in pointCloud:
        contour_count+=1
        for point in contour:
            point.contour_group=contour_count#assign a contour group
            cloud.append(point)
    return cloud


def filter_barrel_lines(camera,angle_range,lidar_vecs,mag_cusion):
    """
    This removes lines the camera sees that are likely attached to a barrel. It groups all camera data into chunks
    determined by size angle_range. If a laser scan is in the angle_range of the camera chunk and the laser scan is
    in front of the camera chunk it pop() the chunk. mag_cusion allows us some room for lines that may be on top of or
    slightly in front of the laser scan; its intent is to buffer camera vibrations that are independent from the chassis.
    :param camera_vecs: list[] of camera_vec objects
    :param angle_range: int  0-10 range reccomended
    :param lidar_vecs: list[] of lidar_vec objects
    :param mag_cusion: int min 300 reccomended
    :return: camera_vecs input data type with barrel noise removed
    """

    camera_vecs = flatten_contours(camera)
    print "camera_vecs: ", camera_vecs
    if len(camera_vecs) == 0:
        return camera_vecs

    else:
        camera_vecs.sort(key=lambda x: x.angle)
        start_iter_angle =int(camera_vecs[0].angle)
        try:
            camera_groups = []
            vec_group = []
            camera_vecs_iterator=iter(camera_vecs)
            angle_start = start_iter_angle
            total_dist=0
            while True:
                next_vec=next(camera_vecs_iterator)
                if next_vec.angle < angle_start + angle_range:#between i and i+range so add to group
                    vec_group.append(next_vec)
                    total_dist+=next_vec.mag
                else:
                    avg_dist = total_dist/len(vec_group)
                    camera_groups.append([angle_start,angle_range,avg_dist,vec_group])
                    angle_start+=angle_range
                    vec_group=[]
                    vec_group.append(next_vec)
                    total_dist=next_vec.mag
        except StopIteration:
            pass

        filtered_camera_groups = camera_groups
        for next_lidar_vec in lidar_vecs:
            for camera_group in camera_groups:
                if int(next_lidar_vec.angle) in range(camera_group[0], camera_group[0] + camera_group[1]):  # check if lidar scan indicates camera data should be thrown
                    #if next_lidar_vec.mag < 2500:#for testing in lab
                    if next_lidar_vec.mag < camera_group[2] + mag_cusion:
                        if camera_group in filtered_camera_groups:
                            filtered_camera_groups.remove(camera_group)
                    break

        filtered_camera_vecs=[]
        for group in filtered_camera_groups:
            for vec in group[3]:
                filtered_camera_vecs.append(vec)

        return filtered_camera_vecs

def update_lidar(laser_pickle):
    global lidar
    lidar=laser_pickle

def vector_to_point(vector):
    return vector.x, vector.y

def vectors_to_points(vector_contours):
    return [[vector_to_point(v) for v in c]for c in vector_contours]

def vectors_to_contours(vectors):#same as points just restores contour grouping
    if len(vectors) ==0:
        return 0
    i =0
    #sort contours by contour group
    vectors_sorted=sorted(vectors,key=lambda x: x.contour_group)
    num_vectors = len(vectors_sorted)-1
    num_contours=vectors_sorted[num_vectors].contour_group

    contours = [[]for j in range(num_contours)]#init a 2d list for contours
    for v in vectors_sorted:
        if v.contour_group == i+1:
            contours[i].append(v)
        else:
            i+=1

    return contours

def calculate_line_angle(contour):
    if contour is None:
        return 0

    x = np.array([v.x for v in contour])
    y = np.array([v.y for v in contour])

    [slope, intercept] = np.polyfit(x, y, 1)
    return math.degrees(math.atan(slope)),slope,intercept


def camera_processor():


    # open a video capture feed
    cam = cv2.VideoCapture(cam_name)

    #init ros & camera stuff
    # pub = rospy.Publisher(topics.CAMERA, String, queue_size=10)
    no_barrel_pub=rospy.Publisher(topics.CAMERA,String, queue_size=10)
    line_angle_pub=rospy.Publisher(topics.LINE_ANGLE, Int16, queue_size=0)
    global lidar
    lidar_obs = rx_subscribe(topics.LIDAR)

    Observable.combine_latest(lidar_obs, lambda n: (n)) \
        .subscribe(update_lidar)

    rospy.init_node('camera')
    rate = rospy.Rate(10)

    rawWidth = 640
    rawHeight = 480
    #camera_info = CameraInfo(53,38,76,91,134)#ground level#half (134 inches out)
    camera_info = CameraInfo(53,40,76,180,217,croppedWidth,croppedHeight)#ground level# 3/4 out
    while not rospy.is_shutdown():


        #grab a frame
        ret_val, img = cam.read()

        #for debugging
        # cv2.line(img,(640/2,0),(640/2,480),color=(255,0,0),thickness=2)
        # cv2.line(img,(0,int(480*.25)),(640,int(480*.25)),color=(255,0,0),thickness=2)

        #crop down to speed processing time
        #img = cv2.imread('test_im2.jpg')
        dim = (rawWidth,rawHeight)
        img=cv2.resize(img,dim,interpolation=cv2.INTER_AREA)
        cropRatio = float(croppedHeight)/float(rawHeight)
        crop_img = img[int(rawHeight * float(1-cropRatio)):rawHeight, 0:rawWidth]  # crops off the top 25% of the image
        cv2.imshow("cropped", crop_img)

        #process the cropped image. returns a "birds eye" of the contours & binary image
        img_displayBirdsEye, contours = process_image(crop_img, camera_info)

        #raw
        contours = convert_to_cartesian(camera_info.map_width, camera_info.map_height, contours)
        #for filtered barrels
        vec2d_contour = contours_to_vectors(contours)#replaces NAV
        filtered_contours = filter_barrel_lines(camera=vec2d_contour, angle_range=4,lidar_vecs=lidar,mag_cusion=300)

        #EXTEND THE LINES
        filtered_cartesian_contours = vectors_to_contours(filtered_contours)

        try:

            closest_filtered_contour = closest_contour(filtered_cartesian_contours)

            # print "CLOSESTCONTOUR: ",closest_filtered_contour

            x_range = 5000
            contour_lines=[]
            interval = 40

            #just one
            line_angle, slope, intercept = calculate_line_angle(closest_filtered_contour)
            for x in range(x_range * -1, x_range):
                if x % interval == 0:
                    y = slope * x + intercept
                    v = Vec2d.from_point(x, y)
                    contour_lines.append(v)

        except TypeError:#no camera data
            contour_lines=[]
            line_angle=0


        #build the camera message with the contours and binary image
        # local_map_msg = CameraMsg(contours=contours, camera_info=camera_info)
        filtered_map_msg=CameraMsg(contours=contour_lines,camera_info=camera_info)

        #make bytestream and pass if off to ros
        # local_map_msg_string = local_map_msg.pickleMe()
        filtered_map_msg_string=filtered_map_msg.pickleMe()

        #rospy.loginfo(local_map_msg_string)
        # pub.publish(local_map_msg_string)
        no_barrel_pub.publish(filtered_map_msg_string)
        line_angle_pub.publish(line_angle)


        if cv2.waitKey(1) == 27:
            break
        rate.sleep()
    cv2.destroyAllWindows()





if __name__ == '__main__':

    # plt.ion()
    # plt.axis([0, 640, 360, 0])
    # animated_plot=plt.plot([],[],'ro')[0]


    # initial values for filters
    BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

    # Median blur
    cv2.namedWindow('img_medianBlur')
    medianRadius = 7
    cv2.createTrackbar('medianRadius', 'img_medianBlur', medianRadius, 20, callback)

    # HSV
    cv2.namedWindow('img_HSV')
    ilowH = 0
    ihighH = 60
    ilowS = 41
    ihighS = 64
    ilowV = 0
    ihighV = 210

    ilowH = 39
    ihighH = 86
    ilowS = 27
    ihighS = 156
    ilowV = 93
    ihighV = 255

    #poolnoodle
    ilowH = 0
    ihighH = 76
    ilowS = 14
    ihighS = 156
    ilowV = 160
    ihighV = 252

    # ilowH = 85
    # ihighH = 118
    # ilowS = 24
    # ihighS = 81
    # ilowV = 122
    # ihighV = 255


    cv2.createTrackbar('lowH', 'img_HSV', ilowH, 255, callback)
    cv2.createTrackbar('highH', 'img_HSV', ihighH, 255, callback)
    cv2.createTrackbar('lowS', 'img_HSV', ilowS, 255, callback)
    cv2.createTrackbar('highS', 'img_HSV', ihighS, 255, callback)
    cv2.createTrackbar('lowV', 'img_HSV', ilowV, 255, callback)
    cv2.createTrackbar('highV', 'img_HSV', ihighV, 255, callback)

    # Gaussian Blur
    cv2.namedWindow('img_gaussianBlur')
    gaussianRadius = 1
    cv2.createTrackbar('gaussianRadius','img_gaussianBlur',gaussianRadius, 20,callback)

    # filter contours
    cv2.namedWindow('img_displayFilteredContours')
    contoursMinArea = 1000
    contoursMinPerimeter = 1
    contoursMinWidth = 0
    contoursMaxWidth = 1000000
    contoursMinHeight = 0
    contoursMaxHeight = 1000000
    contoursSolidity = [21, 100]
    contoursSolidityMin = 21
    contoursSolidityMax = 100
    contoursMaxVertices = 1000000
    contoursMinVertices = 0
    contoursMinRatio = 0
    contoursMaxRatio = 10000

    cv2.createTrackbar('contoursMinArea','img_displayFilteredContours',contoursMinArea,50000, callback)
    cv2.createTrackbar('contoursMinPerimeter','img_displayFilteredContours',contoursMinPerimeter,2000, callback)
    cv2.createTrackbar('contoursMinWidth','img_displayFilteredContours',contoursMinWidth,1000, callback)
    cv2.createTrackbar('contoursMaxWidth','img_displayFilteredContours',contoursMaxWidth,1000, callback)
    cv2.createTrackbar('contoursMinHeight','img_displayFilteredContours',contoursMinHeight,1000, callback)
    cv2.createTrackbar('contoursMaxHeight','img_displayFilteredContours',contoursMaxHeight,1000, callback)
    cv2.createTrackbar('contoursSolidityMin','img_displayFilteredContours',contoursSolidityMin,100, callback)
    cv2.createTrackbar('contoursSolidityMax','img_displayFilteredContours',contoursSolidityMax,100, callback)
    cv2.createTrackbar('contoursMaxVertices','img_displayFilteredContours',contoursMaxVertices,2000, callback)
    cv2.createTrackbar('contoursMinVertices','img_displayFilteredContours',contoursMinVertices,2000, callback)
    cv2.createTrackbar('contoursMinRatio','img_displayFilteredContours',contoursMinRatio,100, callback)
    cv2.createTrackbar('contoursMaxRatio','img_displayFilteredContours',contoursMaxRatio,100, callback)

    # create trackBars for on real time tuning

    croppedWidth = 640
    croppedHeight = 360

    lidar = []

    try:
        camera_processor()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
from camera_msg import CameraMsg
from std_msgs.msg import String
import cv2
import numpy as np
from grip2 import GripPipelineTest
from remove_orange import RemoveOrange
from camera_info import CameraInfo
import math
from enum import Enum
import sys, os
#temp
from guidance import contours_to_vectors
import numpy as np
import matplotlib.pyplot as plt
from operator import itemgetter
import itertools



cam_name = '/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_2B2150DE-video-index0'
CAMERA_NODE = "CAMERA"


def callback(x):
    #the cv2.createTrackbar() requires callback param
    pass

def process_image(img, camera_info):
    #median blur
    medianRadius = cv2.getTrackbarPos('medianRadius', 'img_medianBlur')
    img_medianBlur = blur(src = img,type = BlurType.Median_Filter, radius = medianRadius)
    cv2.namedWindow('img_medianBlur',0)
    #cv2.resizeWindow('img_medianBlur', 640, 480)
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
    cv2.imshow('img_HSV',img_HSV)

    #gaussian blur
    gaussianRadius = cv2.getTrackbarPos('gaussianRadius', 'img_gaussianBlur')
    img_gaussianBlur = blur(src=img_HSV,type=BlurType.Gaussian_Blur,radius=gaussianRadius)
    #cv2.imshow('img_gaussianBlur',img_gaussianBlur)

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
    img_rawContours = find_contours(input=img_gaussianBlur,external_only=False)
    img_displayRawContours=np.ones_like(img)#Return an array of ones with the same shape and type as a given array.
    cv2.drawContours(img_displayRawContours, img_rawContours, -1, (255, 255, 255), thickness=1) #-1 thickness makes them solid
    #cv2.imshow('img_displayRawContours',img_displayRawContours)

    #filter contours
    img_filteredContours = filter_contours(input_contours=img_rawContours,min_area=contoursMinArea,min_perimeter=contoursMinPerimeter,
                                           min_width=contoursMinWidth,max_width=contoursMaxWidth,min_height=contoursMinHeight,
                                           max_height=contoursMaxHeight,solidity=[contoursSolidityMin,contoursSolidityMax],
                                           max_vertex_count=contoursMaxVertices,min_vertex_count=contoursMinVertices,min_ratio=contoursMinRatio,
                                           max_ratio=contoursMaxRatio)
    img_displayFilteredContours = np.ones_like(img)  # Return an array of ones with the same shape and type as a given array.
    cv2.drawContours(img_displayFilteredContours, img_filteredContours, -1, (0, 255, 0), thickness=-1)
    #cv2.imshow('img_displayFilteredContours', img_displayFilteredContours)

    #birds eye view
    img_displayBirdsEye=camera_info.convertToFlat(img_displayFilteredContours)
    cv2.imshow("birdsEye",img_displayBirdsEye)

    #relic of searching traversable space?
    low_green = (0, 240, 0)
    high_green = (255, 255, 255)
    contour_highlight = cv2.inRange(img_displayBirdsEye, low_green,  high_green)
    #cv2.imshow("contour_highlight",contour_highlight)
    img_birdsEye = find_contours(input=contour_highlight,external_only=False)
    img_filteredBirdsEye = filter_contours(input_contours=img_birdsEye,min_area=contoursMinArea,min_perimeter=contoursMinPerimeter,
                                           min_width=contoursMinWidth,max_width=contoursMaxWidth,min_height=contoursMinHeight,
                                           max_height=contoursMaxHeight,solidity=[contoursSolidityMin,contoursSolidityMax],
                                           max_vertex_count=contoursMaxVertices,min_vertex_count=contoursMinVertices,min_ratio=contoursMinRatio,
                                           max_ratio=contoursMaxRatio)
    # if (external_only):
    #     mode = cv2.RETR_EXTERNAL
    # else:
    #     mode = cv2.RETR_LIST
    # method = cv2.CHAIN_APPROX_SIMPLE
    # im2, img_birdsEye, hierarchy = cv2.findContours(contour_highlight,mode=mode,method=method)


    return img_displayBirdsEye, img_filteredContours,img_filteredBirdsEye
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


# def grouper(n, iterable, fillvalue=None):
#     "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
#     args = [iter(iterable)] * n
#     return itertools.izip_longest(fillvalue=fillvalue, *args)


def camera_processor():
    # open a video capture feed
    cam = cv2.VideoCapture(cam_name)

    #init ros stuff
    pub = rospy.Publisher(CAMERA_NODE, String, queue_size=10)
    rospy.init_node(CAMERA_NODE)
    rate = rospy.Rate(10)

    #configure camera
    camera_info = CameraInfo(36.5, 33, 52, 83, 103)
    while not rospy.is_shutdown():
        #grab a frame
        #video
        ret_val, img = cam.read()
        #test imgs
        # img = cv2.imread('test_im1.jpg')
        # width =640
        # height = 480
        # dim = (width,height)
        # img=cv2.resize(img,dim,interpolation=cv2.INTER_AREA)


        #look for our lines and convert them to a birds-eye-view with local_map see camera_msg.py
        img_displayBirdsEye, contours,img_birdContours = process_image(img, camera_info)
        #find contours for birds eye map

        local_map_msg = CameraMsg(local_map_val=img_birdContours, contours=contours, camera_info=camera_info)

        local_map_msg_string = local_map_msg.pickleMe()
        # print(len(local_map_msg_string))
        # rospy.loginfo(local_map_msg_string)
        # print(type(local_map_msg_string))

        #temp

        # for target in img_birdContours:
        # #target = max(contours, key=lambda x: cv2.contourArea(x))
        #     cv2.drawContours(img_displayBirdsEye, [target], -1, [0, 0, 255], -1)  # debug
        #
        #     # #since contours are not in order we need to order them. Y coord pair closest to bender chassis as the first
        #     print "target",target
        #     line = []
        #     for i in range(0,len(target[:][:])):
        #         if target[i][0][1] !=0:#leave out flat areas
        #             line.append([target[i][0][0],target[i][0][1]])
        #     print "line: ",line
        #     sorted_contour = np.array(sorted(line,key = itemgetter(1)))
        #     print "sorted",sorted_contour
        #     print "contour_length: ",len(sorted_contour)
        #     print "next_contour: ", sorted_contour[0]
        #
        #
        #
        #     # grouped = list(grouper(10,sorted_contour))
        #     # print "grouped: ",grouped[0]
        #
        #
        #     windowSize=4
        #     windowCount =0
        #     averagedContours = []
        #     numWindows=int(math.floor(len(sorted_contour)/windowSize))
        #     contourCount = 0
        #     for window in range(0,numWindows):
        #         print"windowSize: ",windowSize
        #         print"numWindows: ",numWindows
        #         #print "whileCOndition: ",windowSize%sorted_contour[contourCount][1]
        #         to_average = []
        #         firstPass = True
        #         try:
        #             while sorted_contour[contourCount][1]%windowSize !=0 or firstPass:#working our way through the next window
        #                 print"enteredLoop"
        #                 print"contourCount: ",contourCount
        #                 print"windowCount: ",windowCount
        #                 to_average.append(sorted_contour[contourCount][0])
        #                 contourCount +=1
        #                 print "To_average: ",to_average
        #                 print "InnerLenToAverage: ",len(to_average)
        #                 firstPass = False
        #             # print "while logic: ",sorted_contour[contourCount][1] % windowSize
        #             newAverage = int(math.floor(sum(to_average) / len(to_average)))
        #             averagedContours.append([newAverage, windowSize * windowCount / 2])
        #             print"averagedContours: ", averagedContours
        #             windowCount += 1
        #         except:
        #             pass
        #
        #
        #     print "Averaged_Contours: ",averagedContours
        #     averagedContours=np.array(averagedContours)
        #
        #     # just example of fitting
        #     xs = sorted_contour[:,0].flatten(order = 'C')
        #     # print"flatenedXlength: ",len(x)
        #     # print "x: ",x
        #     ys = sorted_contour[:,1].flatten(order = 'C')
        #     # print"flatenedYlength: ",len(y)
        #     # print "y: ",y
        #     #print "FlattenedX: ",x
        #
        #     #rebuild list
        #     # fX = []
        #     # fY = []
        #     # for i in range(0,len(x)):
        #     #     if y[i]!=134 and y[i]!=0 and x[i]!=134 and x[i]!=0:
        #     #         fX.append(x[i])
        #     #         fY.append(y[i])
        #     #         line.append([x[i],y[i]])
        #     # print "filteredLengthx: ",len(fX)
        #     # print "filteredLengthy: ",len(fY)
        #     # sX = []
        #     # sY = []
        #     # sorted_contour = sorted(line, key=itemgetter(1))
        #     # print "sorted contour: ",sorted_contour
        #     # for i in range(0,len(sorted_contour)):
        #     #     sX.append(sorted_contour[i][0])
        #     #     sY.append(sorted_contour[i][1])
        #
        #     #betterPlot windowTest?
        #     windowContour=np.array([[188, 0], [201, 10], [194, 20], [200, 30], [190, 40], [198, 50], [195, 60], [197, 70], [195, 80], [196, 90],
        #      [189, 100], [189, 110], [186, 120]])
        #
        #
        #     x = averagedContours[:, 0].flatten(order='C')
        #     # print"flatenedXlength: ",len(x)
        #     # print "x: ",x
        #     y = averagedContours[:, 1].flatten(order='C')
        #
        #
        #     m,b = np.poly1d(np.polyfit(x, y, 1))
        #     ms,bs = np.poly1d(np.polyfit(xs,ys,1))
        #     #print "poly: ",poly
        #     plt.axis([0, 300, 0, 250])
        #     plt.plot([x], [y], 'ro')
        #     plt.plot([xs],[ys],'bo')
        #     for i in range(min(xs),max(xs)):
        #         plt.plot(i,b+m*i,'go')
        #     for i in range(min(xs),max(xs)):
        #         plt.plot(i,bs+ms*i,'yo')
        #     plt.show()
        #
        # cv2.imshow('result',img_displayBirdsEye)

        pub.publish(local_map_msg_string)
        if cv2.waitKey(1) == 27:
            break
    rate.sleep()
    cv2.destroyAllWindows()




if __name__ == '__main__':

    # initial values for filters
    BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

    # Median blur
    cv2.namedWindow('img_medianBlur')
    medianRadius = 5
    cv2.createTrackbar('medianRadius', 'img_medianBlur', medianRadius, 20, callback)

    # HSV
    cv2.namedWindow('img_HSV')
    # ilowH = 0
    # ihighH = 146
    # ilowS = 0
    # ihighS = 43
    # ilowV = 165
    # ihighV = 243

    # ilowH = 90
    # ihighH = 133
    # ilowS = 0
    # ihighS = 83
    # ilowV = 149
    # ihighV = 233

    ilowH = 0
    ihighH = 72
    ilowS = 33
    ihighS = 159
    ilowV = 209
    ihighV = 255

    cv2.createTrackbar('lowH', 'img_HSV', ilowH, 255, callback)
    cv2.createTrackbar('highH', 'img_HSV', ihighH, 255, callback)
    cv2.createTrackbar('lowS', 'img_HSV', ilowS, 255, callback)
    cv2.createTrackbar('highS', 'img_HSV', ihighS, 255, callback)
    cv2.createTrackbar('lowV', 'img_HSV', ilowV, 255, callback)
    cv2.createTrackbar('highV', 'img_HSV', ihighV, 255, callback)

    # Gaussian Blur
    cv2.namedWindow('img_gaussianBlur')
    gaussianRadius = 4
    cv2.createTrackbar('gaussianRadius','img_gaussianBlur',gaussianRadius, 20,callback)

    # filter contours
    cv2.namedWindow('img_displayFilteredContours')
    contoursMinArea = 0
    contoursMinPerimeter = 318
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
    contoursMaxRatio = 1000

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


    try:
        camera_processor()
    except rospy.ROSInterruptException:
        pass
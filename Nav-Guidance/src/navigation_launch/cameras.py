#!/usr/bin/env python
from enum import Enum

import cv2
# temp
import numpy as np
import rospy
from std_msgs.msg import String

import topics
from camera_info import CameraInfo
from camera_msg import CameraMsg

cam_name = '/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_2B2150DE-video-index0'
#cam_name = 2


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
    the point_ is just going through that layer. so don't let it confuse"""


    for contour in contours:
        for point_ in contour:
            for point in point_:#this is the layer in each contour that has the points
                #shift x to center
                # print "OLDPOINT: ", point
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
                #point[1]*=-1
                #shift y down to top of the occlusion box

                #print "SHIFTEDPOINT: ",point

    return contours
    # print contours
    # try:
    #     print "contours0",contours[0][0][:][0]
    #     contours[0][0][0] = 0,0
    #     print "modifiedcontour: ",contours[0][0][0]
    #     print "full contour: ",contours
    # except:
    #     pass



def camera_processor():
    # open a video capture feed
    cam = cv2.VideoCapture(cam_name)

    #init ros & camera stuff
    pub = rospy.Publisher(topics.CAMERA, String, queue_size=10)
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
        cv2.line(img,(640/2,0),(640/2,480),color=(255,0,0),thickness=2)
        cv2.line(img,(0,int(480*.25)),(640,int(480*.25)),color=(255,0,0),thickness=2)

        #crop down to speed processing time
        #img = cv2.imread('test_im2.jpg')
        dim = (rawWidth,rawHeight)
        img=cv2.resize(img,dim,interpolation=cv2.INTER_AREA)
        cropRatio = float(croppedHeight)/float(rawHeight)
        crop_img = img[int(rawHeight * float(1-cropRatio)):rawHeight, 0:rawWidth]  # crops off the top 25% of the image
        cv2.imshow("cropped", crop_img)

        #process the cropped image. returns a "birds eye" of the contours & binary image
        img_displayBirdsEye, contours = process_image(crop_img, camera_info)

        contours = convert_to_cartesian(camera_info.map_width, camera_info.map_height, contours)

        #build the camera message with the contours and binary image
        local_map_msg = CameraMsg(local_map_val=img_displayBirdsEye, contours=contours, camera_info=camera_info)

        #make bytestream and pass if off to ros
        local_map_msg_string = local_map_msg.pickleMe()
        #rospy.loginfo(local_map_msg_string)
        pub.publish(local_map_msg_string)

        #temp



        for target in contours:
        #target = max(contours, key=lambda x: cv2.contourArea(x))
            cv2.drawContours(img_displayBirdsEye, [target], -1, [0, 0, 255], -1)  # debug

            # #since contours are not in order we need to order them. Y coord pair closest to bender chassis as the first
            # print "target",target
            # otherLine = []
            # for i in range(0,len(target[:][:])):
            #     if target[i][0][1] !=0:#leave out flat areas
            #         line.append([target[i][0][0],target[i][0][1]])
            # print "line: ",otherLine
            # sorted_contour = np.array(sorted(otherLine,key = itemgetter(1)))
            # print "sorted",sorted_contour
            # print "contour_length: ",len(sorted_contour)
            # print "next_contour: ", sorted_contour[0]



            # grouped = list(grouper(10,sorted_contour))
            # print "grouped: ",grouped[0]


            # windowSize=4
            # windowCount =0
            # averagedContours = []
            # numWindows=int(math.floor(len(sorted_contour)/windowSize))
            # contourCount = 0
            # for window in range(0,numWindows):
            #     print"windowSize: ",windowSize
            #     print"numWindows: ",numWindows
            #     #print "whileCOndition: ",windowSize%sorted_contour[contourCount][1]
            #     to_average = []
            #     firstPass = True
            #     try:
            #         while sorted_contour[contourCount][1]%windowSize !=0 or firstPass:#working our way through the next window
            #             print"enteredLoop"
            #             print"contourCount: ",contourCount
            #             print"windowCount: ",windowCount
            #             to_average.append(sorted_contour[contourCount][0])
            #             contourCount +=1
            #             print "To_average: ",to_average
            #             print "InnerLenToAverage: ",len(to_average)
            #             firstPass = False
            #         # print "while logic: ",sorted_contour[contourCount][1] % windowSize
            #         newAverage = int(math.floor(sum(to_average) / len(to_average)))
            #         averagedContours.append([newAverage, windowSize * windowCount / 2])
            #         print"averagedContours: ", averagedContours
            #         windowCount += 1
            #     except:
            #         pass
            #
            #
            # print "Averaged_Contours: ",averagedContours
            # averagedContours=np.array(averagedContours)

            # just example of fitting
            # xs = sorted_contour[:,0].flatten(order = 'C')
            # # print"flatenedXlength: ",len(x)
            # # print "x: ",x
            # ys = sorted_contour[:,1].flatten(order = 'C')
            # print"flatenedYlength: ",len(y)
            # print "y: ",y
            #print "FlattenedX: ",x

            #rebuild list
            # fX = []
            # fY = []
            # for i in range(0,len(x)):
            #     if y[i]!=134 and y[i]!=0 and x[i]!=134 and x[i]!=0:
            #         fX.append(x[i])
            #         fY.append(y[i])
            #         line.append([x[i],y[i]])
            # print "filteredLengthx: ",len(fX)
            # print "filteredLengthy: ",len(fY)
            # sX = []
            # sY = []
            # sorted_contour = sorted(line, key=itemgetter(1))
            # print "sorted contour: ",sorted_contour
            # for i in range(0,len(sorted_contour)):
            #     sX.append(sorted_contour[i][0])
            #     sY.append(sorted_contour[i][1])


            # x = averagedContours[:, 0].flatten(order='C')
            # # print"flatenedXlength: ",len(x)
            # # print "x: ",x
            # y = averagedContours[:, 1].flatten(order='C')

            # x = target[:, 0,0].flatten(order='C')
            # # print"flatenedXlength: ",len(x)
            # # print "x: ",x
            # y = target[:, 0,1].flatten(order='C')
            #
            #
            # m,b = np.poly1d(np.polyfit(x, y, 1))
            #ms,bs = np.poly1d(np.polyfit(xs,ys,1))
            #print "poly: ",poly


            # plt.plot([x], [y], 'ro')
            # #plt.plot([xs],[ys],'bo')
            # for i in range(min(x),max(x)):
            #     plt.plot(i,b+m*i,'go')
            # for i in range(min(xs),max(xs)):
            #     plt.plot(i,bs+ms*i,'yo')


            # i = len(x)
            # print "x",x
            # animated_plot.set_xdata(x[0:i])
            # print "xindex: ",x[0:i]
            # i = len(y)
            # animated_plot.set_ydata(y[0:i])





        pub.publish(local_map_msg_string)
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

    ilowH = 49
    ihighH = 64
    ilowS = 43
    ihighS = 206
    ilowV = 147
    ihighV = 255

    # ilowH = 44
    # ihighH = 62
    # ilowS = 24
    # ihighS = 119
    # ilowV = 158
    # ihighV = 252

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

    try:
        camera_processor()
    except rospy.ROSInterruptException:
        pass
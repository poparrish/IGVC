#!/usr/bin/env python
import rospy
from camera_msg import CameraMsg
from std_msgs.msg import String
import cv2
import numpy as np
from grip2 import GripPipelineTest
from camera_info import CameraInfo

'''
Configuration stuff

'''
cam1num = '/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_2B2150DE-video-index0'
# cam1num = 'linepic1.JPG'
grip = GripPipelineTest()

CAMERA_NODE = "CAMERA"

# Green pool noodles
# line_thresh_hue = [39, 64]
# line_thresh_sat = [55, 255]
# line_thresh_val = [0, 255]

# can_traverse_thresh_hue=[0, 40]
# can_traverse_thresh_sat=[0, 155]
# can_traverse_thresh_val=[20, 255]

# # for real values
# line_thresh_hue=[0,  180]
# line_thresh_sat=[0, 20]
# line_thresh_val=[73, 255]


def processImage(img, camera_info):
    line_contours = grip.process(img)
    big_map = np.zeros_like(img)
    cv2.drawContours(big_map, line_contours, -1, (0, 255, 0), thickness = -1)

    local_map = camera_info.convertToFlat(big_map)
    low_green = (0,100,0)
    high_green = (0,255,0)
    
    contour_highlight = cv2.inRange(local_map, low_green,  high_green)
    external_only = False
    if(external_only):
        mode = cv2.RETR_EXTERNAL
    else:
        mode = cv2.RETR_LIST
    method = cv2.CHAIN_APPROX_SIMPLE
    im2, line_contours, hierarchy = cv2.findContours(contour_highlight, mode=mode, method=method)
    
    # cv2.drawContours(local_map, line_contours, -1, (255,255,0), thickness = -1 )
    # good_contours = grip.process(flat_map, hsv_hue_thresh = can_traverse_thresh_hue, hsv_sat_thresh = can_traverse_thresh_sat,hsv_val_thresh = can_traverse_thresh_val)
    # cv2.drawContours(local_map, good_contours, -1, (255, 0, 0), thickness = -1)

    # line_contours = grip.process(flat_map, hsv_hue_thresh = line_thresh_hue, hsv_sat_thresh = line_thresh_sat,hsv_val_thresh = line_thresh_val)
    # cv2.drawContours(local_map, line_contours, -1, (0, 255, 0), thickness = -1)
    cv2.imshow('local_map', local_map)
    
    # cv2.imshow('big_map', big_map)
    # cv2.imshow('flat_map', flat_map)
    return local_map, line_contours


def cameraProcessor():
    cam = cv2.VideoCapture(cam1num)

    pub = rospy.Publisher(CAMERA_NODE, String, queue_size=10)
    rospy.init_node(CAMERA_NODE)
    rate = rospy.Rate(10) # 10hz 
    camera_info = CameraInfo(36.5, 33, 52, 83, 103)
    while not rospy.is_shutdown():
        ret_val, img = cam.read()
        local_map, contours = processImage(img, camera_info)
        # local_map = np.zeros([3,3,3])
        local_map_msg = CameraMsg(local_map_val = local_map, contours = contours, camera_info = camera_info)
        local_map_msg_string = local_map_msg.pickleMe()
        # print(len(local_map_msg_string))
        # rospy.loginfo(local_map_msg_string)
        # print(type(local_map_msg_string))
        pub.publish(local_map_msg_string)
        if cv2.waitKey(1) == 27:
            break
        rate.sleep()
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        cameraProcessor()
    except rospy.ROSInterruptException:
        pass

import rospy
from camera_msg import CameraMsg
from std_msgs.msg import String
import cv2
import numpy as np
from grip import GripPipelineTest

'''
Configuration stuff

'''
cam1num = 0
cam = cv2.VideoCapture(cam1num)
grip = GripPipelineTest()

#Check to see if HSL might be better on accident it seemed like it could do okay
line_thresh_hue=[39,  64]
line_thresh_sat=[55, 255]
line_thresh_val=[0, 255]

can_traverse_thresh_hue=[0, 40]
can_traverse_thresh_sat=[0, 155]
can_traverse_thresh_val=[20, 255]

'''
A class that is in charge of flattening an image from a camera given
a known height of the camera
A known distance to the closest thing the camera can see
the width that is seen at that closest distance
a known distance to the "mioddle" of the screen (halfway up what is seen)
a knowd width that is seen at that middle
It then does 
'''

class CameraInfo:
    def __init__(self, h, d1, w1, d2, w2):
        self.height = h
        self.closedist = d1
        self.closewidth = w1
        self.middist = d2
        self.midwidth = w2
        self.per_inch = 4
        self.map_height = (self.middist - self.closedist) * self.per_inch
        self.map_width = self.midwidth * self.per_inch 
        self.trim = int(((float(self.midwidth - self.closewidth)/2)/float(self.middist - self.closedist))*self.map_height)
    
    def convertToFlat(self, image):
        pts_src = np.float32([[0, 240],[640,240],[0, 480],[640, 480]])
        left_trim = self.trim
        #print(left_trim)
        right_trim = self.map_width - self.trim
        pts_dst = np.float32([[0, 0],[self.map_width,0],[left_trim, self.map_height],[right_trim, self.map_height]])
        mtx = cv2.getPerspectiveTransform(pts_src,pts_dst)
        flat_map = cv2.warpPerspective(image, mtx, (self.map_width, self.map_height))
        
        return flat_map

def processImage(camera_info):
    ret_val, img = cam.read()
    flat_map = camera_info.convertToFlat(img)
    local_map = np.zeros_like(flat_map)

    good_contours = grip.process(flat_map, hsv_hue_thresh = can_traverse_thresh_hue, hsv_sat_thresh = can_traverse_thresh_sat,hsv_val_thresh = can_traverse_thresh_val)
    cv2.drawContours(local_map, good_contours, -1, (255, 0, 0), thickness = -1)

    line_contours = grip.process(flat_map, hsv_hue_thresh = line_thresh_hue, hsv_sat_thresh = line_thresh_sat,hsv_val_thresh = line_thresh_val)
    cv2.drawContours(local_map, line_contours, -1, (0, 255, 0), thickness = -1)
    # cv2.imshow('local_map', local_map)
    # cv2.imshow('flat_map', flat_map)
    return local_map

def cameraProcessor():
    pub = rospy.Publisher('cameraMsgSent', String, queue_size=10)
    rospy.init_node('camera_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    camera_info = CameraInfo(36.5, 33, 52, 83, 103)        
    while not rospy.is_shutdown():
        local_map  = processImage(camera_info)
        local_map_msg = CameraMsg(local_map)
        local_map_msg_string = local_map_msg.pickleMe()
        #print(len(local_map_msg_string))
        # rospy.loginfo(local_map_msg_string)
        pub.publish(local_map_msg_string)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
        rate.sleep()
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        cameraProcessor()
    except rospy.ROSInterruptException:
        pass

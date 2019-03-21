import cv2
import numpy as np

'''
A class that is in charge of flattening an image from a camera given
a known height of the camera
A known distance to the closest thing the camera can see
the width that is seen at that closest distance
a known distance to the "mioddle" of the screen (halfway up what is seen)
a knowd width that is seen at that middle
It then be wizard
'''


class CameraInfo:
    def __init__(self, h, d1, w1, d2, w2,fx,fy):
        self.height = h
        self.closedist = d1
        self.closewidth = w1
        self.middist = d2
        self.midwidth = w2
        self.per_inch = 1
        self.map_height = fy
        self.map_width = fx
        self.trim = int(((float(self.midwidth - self.closewidth)/2)/float(self.middist - self.closedist))*self.map_height)
        self.rectangle_height = int(.88*self.map_height)#number is ratio for how far down the frame rectangle should start

    def convertToFlat(self, image):
        pts_src = np.float32([[0, int(self.map_height)], [self.map_width, int(self.map_height)], [0, 0], [self.map_width, 0]])
        left_trim = self.trim
        right_trim = self.map_width - self.trim
        pts_dst = np.float32([[left_trim, self.map_height],[right_trim, self.map_height],[0, 0],[self.map_width,0]])
        mtx = cv2.getPerspectiveTransform(pts_src,pts_dst)
        flat_map = cv2.warpPerspective(image, mtx, (self.map_width, self.map_height))
        cv2.rectangle(flat_map, (60+left_trim,self.rectangle_height), (right_trim-60,self.map_height), (0,0,0), -1)#occlude bender chassis
        return flat_map


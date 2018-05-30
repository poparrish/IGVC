import cv2
import numpy as np

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
        wx = len(image[0])
        hy = len(image)
        pts_src = np.float32([[0, int(hy*.33)], [wx, int(hy*.33)], [0, hy], [wx, hy]])
        left_trim = self.trim
        #print(left_trim)
        right_trim = self.map_width - self.trim
        pts_dst = np.float32([[0, 0],[self.map_width,0],[left_trim, self.map_height],[right_trim, self.map_height]])
        mtx = cv2.getPerspectiveTransform(pts_src,pts_dst)
        flat_map = cv2.warpPerspective(image, mtx, (self.map_width, self.map_height))
        cv2.rectangle(flat_map, (155,135), (260,200), (0,0,0), -1) 
        return flat_map


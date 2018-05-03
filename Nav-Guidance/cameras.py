import rospy
from camera_msg import CameraMsg
from std_msgs.msg import String
import cv2
import numpy as np
from grip import GripPipelineTest
proc = GripPipelineTest()
cam1num = 1
cam = cv2.VideoCapture(cam1num)

class CameraInfo(self):
    def __init__(self, h, d1, w1, d2, w2):
        self.height = h
        self.closedist = d1
        self.closewidth = w1
        self.middist = d2
        self.midwidth = w2
    
    def covertToFlat(self, image):
        newh = len(image)*2
        neww = len(image[0])*2

def processImage():
    img = cam.read()
    countours = proc.process(img)
    for c in countours:
        pass
    return CameraMsg()

def cameraProcessor():
    pub = rospy.Publisher('cameraMsgSent', String, queue_size=10)
    rospy.init_node('camera_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        new_camera_msg = processImage()
        new_camera_msg_string = new_camera_msg.pickleMe()
        rospy.loginfo(new_camera_msg_string)
        pub.publish(new_camera_msg_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        cameraProcessor()
    except rospy.ROSInterruptException:
        pass



def show_webcam():
    cam = cv2.VideoCapture('videoplayback.mp4')
    while True:
        ret_val, img = cam.read()
        contours = proc.process(img, hsv_val_thresh = [130,255], hsv_sat_thresh = [0,120])

        for low in [1]:
            countours = proc.process(img)
            counter = 0
            for c in countours:
                counter = 0
                points = []
                for p in c:
                    p = p[0]
                    points.append([p[0], p[1]])
                cv2.polylines(img, np.int32([points]), True, (0,0,255))
            print((len(img),len(img[0])))

        cv2.imshow('my webcam', img)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()


def main():
    show_webcam()


if __name__ == '__main__':
    main()

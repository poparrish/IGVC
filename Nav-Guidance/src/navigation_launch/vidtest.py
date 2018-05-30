import glob
from grip2 import GripPipelineTest
import cv2
from cameras import processImage
from camera_info import CameraInfo 

grip = GripPipelineTest()
images = []
camera_info = CameraInfo(36.5, 33, 52, 83, 103)
cam = cv2.VideoCapture('test.mp4')
while(cap.isOpened()):
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

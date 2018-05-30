import glob
from grip2 import GripPipelineTest
import cv2
from cameras import processImage
from camera_info import CameraInfo 

grip = GripPipelineTest()
images = []
img = "test"
camera_info = CameraInfo(36.5, 33, 52, 83, 103)
cam1num = '/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_2B2150DE-video-index0'
cam = cv2.VideoCapture(cam1num)
while True:
    ret, frame = cam.read()
    result = grip.process(frame)
    lm, lc = processImage(frame, camera_info)
    cv2.drawContours(frame, result, -1, (0,255,0), 3)
    cv2.imshow(img, frame)
    cv2.imshow(img + "from cameras", lm)
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()

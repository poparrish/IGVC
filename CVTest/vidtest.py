import glob
from grip2 import GripPipelineTest
import cv2
from cameras import processImage
from camera_info import CameraInfo 

grip = GripPipelineTest()
images = []
img = "test"
camera_info = CameraInfo(36.5, 33, 52, 83, 103)
cam = cv2.VideoCapture('test.mp4')
while(cam.isOpened()):
    ret, frame = cam.read()
    for i in xrange(15):
        cam.read()
    result = grip.process(frame)
    lm, lc = processImage(frame, camera_info)
    cv2.drawContours(frame, result, -1, (0,255,0), 3)
    cv2.imshow(img, frame)
    cv2.imshow(img + "from cameras", lm)
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.waitKey(1) & 0xFF == ord('p'):
        while True:
            if cv2.waitKey(1) & 0xFF == ord('n'):
                break
            pass


cam.release()
cv2.destroyAllWindows()

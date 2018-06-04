import glob
from grip2 import GripPipelineTest
import cv2
from cameras import processImage
from camera_info import CameraInfo 

grip = GripPipelineTest()
images = []
img = "test"
camera_info = CameraInfo(36.5, 33, 52, 83, 103)
cam = cv2.VideoCapture('1130IGVC.mp4')
while(cam.isOpened()):
    ret, frame = cam.read()
    # cv2.imshow("image", frame)
    for i in xrange(5):
        cam.read()
    result = grip.process(frame)
    lm, lc = processImage(frame, camera_info)
    cv2.drawContours(frame, result, -1, (0,255,0), 3)
    # cv2.imshow(img, frame)
    # cv2.imshow(img + "from cameras", lm)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()

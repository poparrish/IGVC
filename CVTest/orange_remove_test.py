import glob
from grip2 import GripPipelineTest
from remove_orange import RemoveOrange
import cv2
from cameras import processImage
from camera_info import CameraInfo 

grip = GripPipelineTest()
orange = RemoveOrange()
images = []
img = "test"
camera_info = CameraInfo(36.5, 33, 52, 83, 103)

for img in glob.glob("*.jpg"):
    readimage= cv2.imread(img)
    processImage(readimage, camera_info)
    # cv2.imshow(img, readimage)
    while True:
        if cv2.waitKey() == 27:
            break

cv2.destroyAllWindows()

import glob
from grip2 import GripPipelineTest
import cv2
from cameras import processImage
from camera_info import CameraInfo 

grip = GripPipelineTest()
images = []
camera_info = CameraInfo(36.5, 33, 52, 83, 103)
for img in glob.glob("*.JPG"):
    readimage= cv2.imread(img)
    result = grip.process(readimage)
    print(img)
    print(type(result))
    lm, lc = processImage(readimage, camera_info)
    cv2.drawContours(readimage, result, -1, (0,255,0), 3)
    cv2.imshow(img, readimage)
    cv2.imshow(img + "from cameras", lm)
    while True:
    	if cv2.waitKey() == 27:
    		break
    cv2.destroyAllWindows() 
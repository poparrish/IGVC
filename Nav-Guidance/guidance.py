import pickle
import rospy
import cv2
from camera_msg import CameraMsg
from cameras import CAMERA_NODE
from gps import GPS_NODE
from lidar import LIDAR_NODE
from nav_msg import NavMsg
from nav import NAV_NODE
from std_msgs.msg import String
import time

GUIDANCE_NODE = "GUIDANCE"
GUIDANCE_HZ = 5
class Guidance():
    def __init__(self):
        self.translation_vector = None
        self.rotational_vector  = None
        self.new_guidance = False

    def processNavState(self, navState):
        self.new_guidance = False

    def unpickleNavMsg(self, data):
        self.new_guidance = True
        step1ns = pickle.loads(data.data)
        print(step1ns['gps'])
        print(type(step1ns))


def findWhiteLines(image):
    pass

def main():
    guidance = Guidance()
    rospy.init_node(GUIDANCE_NODE, anonymous=True)
    rospy.Subscriber(NAV_NODE, String, guidance.unpickleNavMsg)
    guidance_publisher = rospy.Publisher(NAV_NODE, String, queue_size = 10)
    rate = rospy.Rate(GUIDANCE_HZ)
    while not rospy.is_shutdown():
        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

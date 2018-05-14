import pickle
import rospy
import cv2
import sys
from camera.camera_msg import CameraMsg
from camera.cameras import CAMERA_NODE
sys.modules['CameraMsg'] = CameraMsg
from lidar.lidar import LIDAR_NODE
from nav_msg import NavMsg
from std_msgs.msg import String
import time

print(CAMERA_NODE)

class Navigation():
    def __init__(self):
        self.new_camera_msg = None
        self.new_lidar_msg = None
        self.new_gps_msg = None
            
    def unpickleCameraMsg(self, unpickleString):
        print('recieved unpickleString')
        unpickleString = unpickleString.data
        self.new_camera_msg = CameraMsg(pickled_values = unpickleString)

    def getCameraMsg(self):
        return self.new_camera_msg

    def unpickleLidarMsg(self, unpickleString):
        print('recieved unpickleString')
        unpickleString = unpickleString.data
        self.new_lidar_msg = LidarMsg(pickled_values = unpickleString)

    def getLidarMsg(self):
        return self.new_lidar_msg

    def unpickleGpsMsg(self, unpickleString):
        print('recieved unpickleString')
        unpickleString = unpickleString.data
        self.new_gps_msg = GpsMsg(pickled_values = unpickleString)

    def getGpsMsg(self):
        return self.new_gps_msg

    def noneOutMsg(self):
        self.new_camera_msg = None
        self.new_lidar_msg = None
        self.new_gps_msg = None

    def processState():
        if self.new_camera_msg == None:
            return
        print("storing camera msg")


def main():
    nav = Navigation()
    rospy.init_node('nav_node', anonymous=True)
    rospy.Subscriber(CAMERA_NODE, String, nav.unpickleCameraMsg)
    rospy.Subscriber(LIDAR_NODE, String, nav.unpickleCameraMsg)
    rospy.Subscriber('gpsMsgSent', String, nav.unpickleCameraMsg)
    guidance_publisher = rospy.Publisher('navigationMsgSent', String, queue_size = 10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        new_nav_msg = NavMsg()
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
        
        if nav.getCameraMsg() != None:
            print(len(nav.getCameraMsg().getLocalMap()))
            local_map = nav.getCameraMsg().getLocalMap()
            cv2.imshow('local_map', local_map)
        #if there isn't a new update publish the empty string so the guidance node knows it isn't going to get anything new
        new_nav_msg_string = ""
        if(new_nav_msg != None):
            new_nav_msg_string = new_nav_msg.pickleMe()
        guidance_publisher.publish(new_nav_msg_string)
        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

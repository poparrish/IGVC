
import rospy
import cv2
from camera_msg import CameraMsg
from std_msgs.msg import String
import time

new_camera_msg = None
new_lidar_msg = None
new_gps_msg = None

def unpickleCameraMsg(unpickleString):
    # print('recieved unpickleString')
    # print(unpickleString.data)
    unpickleString = unpickleString.data
    # print(type(unpickleString))
    new_camera_msg = CameraMsg(pickled_values = unpickleString)
    
    # print(new_camera_msg.getLocalMap().shape)

def processState():
    # print("in processState")
    if new_camera_msg == None:
        # print('cam msg none')
        return

    local_map = new_camera_msg.getLocalMap()
    print(local_map)
    cv2.imshow('local_map', local_map)
    
def navigation():
    rospy.init_node('nav_node', anonymous=True)
    rospy.Subscriber('cameraMsgSent', String, unpickleCameraMsg)
    guidance_publisher = rospy.Publisher('navigationMsgSent', String, queue_size = 10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        new_nav_msg = processState()
        #None everything out so that way if there isn't a new update since the last used update it wont be used again
        
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
        
        #if there isn't a new update publish the empty string so the guidance node knows it isn't going to get anything new
        new_nav_msg_string = ""
        if(new_nav_msg != None):
            new_nav_msg_string = new_nav_msg.pickleMe()
        guidance_publisher.publish(new_nav_msg_string)
        rate.sleep()
        # new_camera_msg = None
        new_lidar_msg = None
        new_gps_msg = None
    cv2.destroyAllWindows()

if __name__ == '__main__':
    navigation()

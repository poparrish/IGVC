
import rospy
from camera_msg import CameraMsg
from std_msgs.msg import String

new_camera_msg = None
new_lidar_msg = None
new_gps_msg = None

def unpickleCameraMsg(unpickleString):
    new_camera_msg = CameraMsg(unpickleString)

def processState():
    if new_camera_msg == None:
        return
    
def navigation():
    rospy.Subscriber('cameraMsgSent', String, unpickleCameraMsg)
    guidance_publisher = rospy.Publisher('navigationMsgSent', String, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        new_nav_msg = processState()
        new_nav_msg_string = new_nav_msg.pickleMe()
        guidance_publisher.publish(new_nav_msg_string)
        #None everything out so that way if there isn't a new update since the last used update it wont be used again
        new_camera_msg = None
        new_lidar_msg = None
        new_gps_msg = None
        rate.sleep()

if __name__ == '__main__':
    navigation()

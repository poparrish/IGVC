import rospy
from camera_msg import CameraMsg
from std_msgs.msg import String

def processImage():
    return CameraMsg()

def cameraProcessor():
    pub = rospy.Publisher('cameraMsgSent', String, queue_size=10)
    rospy.init_node('camera_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        new_camera_msg = processImage()
        new_camera_msg_string = new_camera_msg.pickleMe()
        rospy.loginfo(new_camera_msg_string)
        pub.publish(new_camera_msg_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        cameraProcessor()
    except rospy.ROSInterruptException:
        pass

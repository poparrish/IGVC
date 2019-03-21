import pickle
import rospy
import cv2
from camera_msg import CameraMsg
import topics
from std_msgs.msg import String

NAV_HZ = 5
NAV_NODE = 'NAV'
class Navigation():
    def __init__(self):
        self.new_camera_msg = None
        self.new_lidar_msg = None
        self.new_gps_msg = None
            
    def unpickleCameraMsg(self, data):
        # print('Recieved Camera')
        unpickleString = data.data
        self.new_camera_msg = CameraMsg(pickled_values = unpickleString)

    def getCameraMsg(self):
        return self.new_camera_msg

    def unpickleLidarMsg(self, data):
        # print('recieved LIDAR')
        unpickleString = data.data
        self.new_lidar_msg = pickle.loads(unpickleString)
        # print(self.new_lidar_msg)
        # self.new_lidar_msg = LidarMsg(pickled_values = unpickleString)

    def getLidarMsg(self):
        return self.new_lidar_msg

    def unpickleGpsMsg(self, data):
        # print('recieved GPS')
        unpickleString = data.data
        # self.new_gps_msg = GpsMsg(pickled_values = unpickleString)

    def getGpsMsg(self):
        return self.new_gps_msg

    def noneOutMsg(self):
        self.new_camera_msg = None
        self.new_lidar_msg = None
        self.new_gps_msg = None

    def processState(self):
        return{'camera': pickle.dumps(self.new_camera_msg), 'gps': pickle.dumps(self.new_gps_msg), 'lidar': pickle.dumps(self.new_lidar_msg)}

def main():
    nav = Navigation()
    rospy.init_node(NAV_NODE, anonymous=True)
    rospy.Subscriber(topics.CAMERA, String, nav.unpickleCameraMsg)
    rospy.Subscriber(topics.GPS, String, nav.unpickleGpsMsg)
    rospy.Subscriber(topics.LIDAR, String, nav.unpickleLidarMsg)
    nav_publisher = rospy.Publisher(NAV_NODE, String, queue_size = 10)
    rate = rospy.Rate(NAV_HZ)
    while not rospy.is_shutdown():
        # new_nav_msg = NavMsg()
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
        
        if nav.getCameraMsg() != None:
            print(len(nav.getCameraMsg().getLocalMap()))
            local_map = nav.getCameraMsg().getLocalMap()
            cv2.imshow('local_map', local_map)
        #if there isn't a new update publish the empty string so the guidance node knows it isn't going to get anything new
        
        nav_publisher.publish(pickle.dumps(nav.processState()))
        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

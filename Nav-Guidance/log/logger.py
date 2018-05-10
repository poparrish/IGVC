import os
import pickle
from datetime import datetime

import rospy
from std_msgs.msg import String

from gps import GPS_NODE
from lidar import LIDAR_NODE


def log_callback(log_file, start_date, node):
    """generates a callback function that logs the data returned from a node"""
    return lambda data: pickle.dump({
        'time': (datetime.now() - start_date).total_seconds(),
        'node': node,
        'data': data.data
    }, log_file)


def logger_start():
    rospy.init_node('LOGGER')

    start = datetime.now()
    log_dir = "./logs"

    try:
        os.makedirs(log_dir)
    except:
        pass

    log_file = open("%s/%s.txt" % (log_dir, start), "w")

    rospy.Subscriber(LIDAR_NODE, String, log_callback(log_file, start, LIDAR_NODE))
    rospy.Subscriber(GPS_NODE, String, log_callback(log_file, start, GPS_NODE))

    rospy.spin()
    log_file.close()


if __name__ == '__main__':
    logger_start()

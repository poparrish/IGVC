import os
import pickle
from datetime import datetime

import rospy
from std_msgs.msg import String

from lidar import LIDAR_NODE

NODES_TO_LOG = {
    LIDAR_NODE: String,
    'cameraMsgSent': String
}


def log_callback(log_file, start_date, node):
    """generates a callback function that logs the data returned from a node"""
    return lambda data: pickle.dump({
        'time': (datetime.now() - start_date).total_seconds(),
        'node': node,
        'data': data.data
    }, log_file)


def start_logger():
    rospy.init_node('LOGGER')

    start = datetime.now()
    log_dir = "./logs"

    try:
        os.makedirs(log_dir)
    except:
        pass

    log_file = open("%s/%s.txt" % (log_dir, start), "w")

    # Write the replay "header"
    pickle.dump({
        'nodes': NODES_TO_LOG.keys()
    }, log_file)

    for node, data_class in NODES_TO_LOG.iteritems():
        rospy.Subscriber(node, data_class, log_callback(log_file, start, node))

    rospy.spin()
    log_file.close()


if __name__ == '__main__':
    start_logger()

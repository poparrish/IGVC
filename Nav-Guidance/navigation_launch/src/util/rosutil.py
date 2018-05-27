import pickle

import rospy
from rx.subjects import ReplaySubject
from std_msgs.msg import String


def rx_subscribe(node, data_class=String, parse=pickle.loads, buffer_size=0):
    source = ReplaySubject(buffer_size)
    rospy.Subscriber(node, data_class, lambda msg: source.on_next(parse(msg.data)))
    return source.as_observable()

import pickle

import rospy
from rx.subjects import ReplaySubject
from std_msgs.msg import String


def unpickle(msg):
    return pickle.loads(msg.data)


def rx_subscribe(node, data_class=String, parse=unpickle, buffer_size=0):
    source = ReplaySubject(buffer_size)
    rospy.Subscriber(node, data_class, lambda msg: source.on_next(parse(msg) if parse is not None else msg))
    return source.as_observable()

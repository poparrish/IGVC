import pickle

import rospy
from rx import Observable
from rx.subjects import ReplaySubject
from std_msgs.msg import String


def unpickle(msg):
    return pickle.loads(msg.data)


# TODO: Get rid of parse...
def rx_subscribe(node, data_class=String, parse=unpickle, buffer_size=1):
    source = ReplaySubject(buffer_size)
    rospy.Subscriber(node, data_class, lambda msg: source.on_next(parse(msg) if parse is not None else msg),
                     queue_size=buffer_size)
    return source.as_observable()


def extract_tf(child_frame_id):
    def match_frame(msg):
        matches = [t for t in msg.transforms if t.child_frame_id == child_frame_id]
        return Observable.from_(matches)

    return lambda obs: obs.flat_map(match_frame)

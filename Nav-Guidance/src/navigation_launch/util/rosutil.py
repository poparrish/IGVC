import pickle

import rospy
from rx import Observable, AnonymousObservable
from rx.concurrency import ThreadPoolScheduler
from rx.internal import extensionmethod
from rx.subjects import ReplaySubject
from std_msgs.msg import String


def unpickle(msg):
    return pickle.loads(msg.data)


def rx_subscribe(node, data_class=String, parse=unpickle, buffer_size=0):
    source = ReplaySubject(buffer_size)
    rospy.Subscriber(node, data_class, lambda msg: source.on_next(parse(msg) if parse is not None else msg))
    return source.as_observable()


@extensionmethod(Observable)
def exhaust_map(self, selector, scheduler=None):
    scheduler = scheduler or ThreadPoolScheduler(max_workers=1)

    source = self

    def subscribe(observer):
        curr = {'obs': None}

        def on_next(value):
            def start():
                try:
                    return selector(value)
                finally:
                    curr['obs'] = None

            if curr['obs'] is None:
                curr['obs'] = Observable.start(start, scheduler)

        return source.subscribe(on_next, observer.on_error, observer.on_completed)

    return AnonymousObservable(subscribe)

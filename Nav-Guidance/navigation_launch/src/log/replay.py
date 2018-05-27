import pickle
import subprocess
import sys
import time
from sets import Set

import rospy
from std_msgs.msg import String


def iterate_messages(log_file):
    with open(log_file, 'r') as f:
        pickle.load(f)  # skip header

        while True:
            try:
                yield pickle.load(f)
            except EOFError:  # end of replay
                return


def replay_node(node, log_file, start_time=0.0):
    # initialize the node
    rospy.init_node(node)
    pub = rospy.Publisher(node, String, queue_size=20)

    last_time = start_time
    for m in iterate_messages(log_file):

        # publish messages for this node only
        if m['time'] < start_time or m['node'] != node:
            continue

        # sleep, then publish
        time.sleep(m['time'] - last_time)
        last_time = m['time']

        print('publishing to "%s" at %s' % (node, last_time))
        pub.publish(m['data'])


def start_replay(log_file, node=None):
    if node is None:
        with open(log_file, 'r') as f:
            header = pickle.load(f)
            nodes = Set(header['nodes'])

        # spawn fake nodes as their own subprocesses
        procs = []
        for node in nodes:
            print('starting node "%s"' % node)
            p = subprocess.Popen([sys.executable, './replay.py', log_file, node])
            procs.append(p)

        print('replay finished, exit codes: %s' % [p.wait() for p in procs])
    else:
        replay_node(node, log_file)


def arg(index, default=None):
    return args[index] if len(args) > index else default


if __name__ == '__main__':
    args = sys.argv[1:]

    start_replay(log_file=arg(0), node=arg(1))

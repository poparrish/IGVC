import os
import sys
import yaml

import cv2

CONFIG_FILE = os.path.dirname(os.path.realpath(__file__)) + '/default.yaml'


class Trackbar:
    trackbars = []

    def __init__(self, winname, trackbar):
        self.window = winname
        self.name = trackbar
        Trackbar.trackbars.append(self)


def setting_name(window, name):
    return window + '.' + name


def callback(x):
    cfg = {}
    for trackbar in Trackbar.trackbars:
        pos = cv2.getTrackbarPos(trackbar.name, trackbar.window)
        cfg[setting_name(trackbar.window, trackbar.name)] = pos
    with open(CONFIG_FILE, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False)


def load_initial_config():
    try:
        with open(CONFIG_FILE, 'r') as f:
            return yaml.load(f)
    except IOError as e:
        print >> sys.stderr, 'Failed to load camera config %s' % e
        return {}


config = None


def create_persistent_trackbar(name, window, default, count=255):
    global config
    if config is None:
        config = load_initial_config()

    default = config.get(setting_name(window, name), default)
    cv2.createTrackbar(name, window, default, count, callback)
    return Trackbar(window, name)

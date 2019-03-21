#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import topics
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from util import rx_subscribe

map_data = None


def safe_get(img, x, y):
    if 0 <= x < len(img) and 0 <= y < len(img[0]):
        return img[x][y]
    else:
        return 65535  # TODO


def neighbors(x, y):
    yield x + 1, y
    yield x - 1, y
    yield x, y + 1
    yield x, y - 1


def create_path(img, pose):
    cv2.imshow("Image window", img)
    cv2.waitKey(3)

    scale = (MAP_SIZE_PIXELS / 2.0) / (MAP_SIZE_METERS / 2.0)
    position = pose.pose.position
    x = int((MAP_SIZE_PIXELS / 2.0) - position.x * scale)
    y = int((MAP_SIZE_PIXELS / 2.0) - position.y * scale)

    poses = []
    current = safe_get(img, x, y)
    print 'start ', x, y, current
    while True:
        best = min(neighbors(x, y), key=lambda (x, y): safe_get(img, x, y))
        lowest = safe_get(img, best[0], best[1])
        # TODO: Heuristic? A-star?
        if lowest < current:
            current = lowest
            x = best[0]
            y = best[1]
            poses.append(PoseStamped(header=Header(frame_id='map'),
                                     pose=Pose(position=Point(x=-x * 1.0 / scale + (MAP_SIZE_METERS / 2.0),
                                                              y=-y * 1.0 / scale + (MAP_SIZE_METERS / 2.0)))))
        else:
            break

    return Path(header=Header(frame_id='map'), poses=poses)


def start():
    rospy.init_node('path_planning')

    bridge = CvBridge()
    pub = rospy.Publisher(topics.PATH, Path, queue_size=1)

    potential_field = rx_subscribe(topics.POTENTIAL_FIELD, Image, lambda data: bridge.imgmsg_to_cv2(data, "mono16"))
    position = rx_subscribe(topics.MAP_POSE, PoseStamped, None)

    potential_field.with_latest_from(position, create_path).subscribe(pub.publish)

    rospy.spin()


if __name__ == '__main__':
    start()

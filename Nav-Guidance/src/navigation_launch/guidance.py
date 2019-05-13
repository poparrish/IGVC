#!/usr/bin/env python
import math

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, PoseArray, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from rx import Observable
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage

import topics
from guidance import calculate_line_angle, compute_potential
from guidance.attractor_placement import generate_path
from guidance.gps_guidance import dist_to_waypoint, calculate_gps_heading
from map_msg import MapData
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from nav import rx_subscribe
from util import Vec2d, avg, to180
from util.rosutil import extract_tf

GUIDANCE_NODE = "GUIDANCE"
GUIDANCE_HZ = 10

#
# Drivetrain
#

INITIAL_SPEED = 0.75  # gotta go FAST
INITIAL_DISTANCE_CUTOFF = 5000  # slow down once we're within 5m of something

NORMAL_SPEED = .5  # forward speed in m/s
MAX_ROTATION = 30  # max crabbing angle
MAX_TRANSLATION = 90  # max crabbing angle

control = None


def update_drivetrain(translation, rotation, speed):
    translation = max(-MAX_TRANSLATION, min(translation, MAX_TRANSLATION))
    rotation = max(-MAX_ROTATION, min(rotation, MAX_ROTATION))
    # speed, velocity_vector, theta_dot
    control.publish(Vector3(x=speed, y=translation, z=rotation))


#
# Waypoints
#

GPS_BUFFER = 5  # buffer GPS messages to help with accuracy

FIRST_WAYPOINT_TOLERANCE = 1  # when to start tracking the first waypoint
WAYPOINT_TOLERANCE = .5  # precision in meters

# WAYPOINTS = [
#    (43.600314, -116.197164),  # N/W corner of field
#    (43.600248, -116.196955),  # center of field
#    (43.600314, -116.197164),  # N/W corner of field
# ]

test1stwp = (42.6785268, -83.1953824)
test2ndwp = (42.6786267, -83.1948953)
# WAYPOINTS = [
#     test2ndwp,
#     test1stwp,
#     test2ndwp
# ]


endof = (42.678363, -83.1945975)
qual1 = (42.6782191223, -83.1955080989)
qual2 = (42.6778987274, -83.1954820799)

rolling_average = []

WAYPOINTS = [
    endof,
    qual1,
    qual2,
    qual1
]

northwp = [42.6790984, -83.1949250546]
midwp = [42.6789603912, -83.1951132036]
southwp = [42.6788026, -83.1949093082]
WAYPOINTS = [
    northwp,
    midwp,
    southwp
]


# WAYPOINTS = [
#     southwp,
#     midwp,
#     northwp
# ]

def reached_waypoint(num, gps_buffer, tolerance):
    waypoint = WAYPOINTS[num]
    distance = avg([dist_to_waypoint(loc, waypoint) for loc in gps_buffer])
    # state_debug.publish(str(distance))
    return distance < tolerance


#
# State machine
#

LINE_FOLLOWING = 'LINE_FOLLOWING'
WAYPOINT_TRACKING = 'WAYPOINT_TRACKING'

DEFAULT_STATE = {
    'state': LINE_FOLLOWING,
    'speed': INITIAL_SPEED,
    'tracking': 0
}
# DEFAULT_STATE = {
#     'state': WAYPOINT_TRACKING,
#     'speed': INITIAL_SPEED,
#     'tracking': 0
# }

debug = None
heading_debug = rospy.Publisher(topics.POTENTIAL_FIELD, PoseStamped, queue_size=1)
costmap_debug = rospy.Publisher('guidance/costmap', Image, queue_size=1)
path_debug = rospy.Publisher('guidance/path', Path, queue_size=1)
bridge = CvBridge()


def compute_next_state(state, (nav, gps_buffer)):
    """guidance state machine"""
    # check if we need to slow down
    speed = state['speed']
    # if speed == INITIAL_SPEED:
    #     clusters = partition(nav['lidar'], cluster_mm=500)
    #     if len(clusters) > 0:
    #         closest = min([c for c in clusters], key=lambda v: v.mag)
    #     else:
    #         closest = 100000000
    #     if closest < INITIAL_DISTANCE_CUTOFF:
    #         speed = NORMAL_SPEED
    #         state['speed'] = speed

    if state['state'] == LINE_FOLLOWING:

        # if we're within range of the first waypoint, start tracking it
        if reached_waypoint(0, gps_buffer, tolerance=FIRST_WAYPOINT_TOLERANCE):
            rospy.loginfo('Begin tracking first waypoint')
            return {
                'state': WAYPOINT_TRACKING,
                'speed': state['speed'],
                'tracking': 0
            }

    if state['state'] == WAYPOINT_TRACKING:
        tracking = state['tracking']

        # if we've reached the current waypoint, start tracking the next one
        if reached_waypoint(tracking, gps_buffer, tolerance=WAYPOINT_TOLERANCE):

            # ... unless we are at the last one, in which case we should resume normal navigation
            if tracking == len(WAYPOINTS) - 1:
                rospy.loginfo('Reached all waypoints, resuming normal operation')
                return {
                    'state': LINE_FOLLOWING,
                    'speed': state['speed'],
                }

            next = tracking + 1
            rospy.loginfo('Begin tracking waypoint %s', next)
            return {
                'state': WAYPOINT_TRACKING,
                'speed': state['speed'],
                'tracking': next
            }

    return state


scale = MAP_SIZE_METERS / float(MAP_SIZE_PIXELS)


def x_to_m(x):
    """converts x (pixel coordinate) to world coordinate"""
    return ((MAP_SIZE_PIXELS / -2.0) + x) * scale


def y_to_m(y):
    """converts y (pixel coordinate) to world coordinate"""
    return ((MAP_SIZE_PIXELS / 2.0) - y) * scale


def update_control((msg, map_grid, map_pose, pose, state)):
    """figures out what we need to do based on the current state and map"""
    camera = msg['camera']
    lidar = msg['lidar']
    # for v in lidar:
    #     if v.angle > 345 or v.angle < 15:
    #         print(v)
    gps = msg['gps']

    transform = pose.transform.translation
    map_transform = map_pose.transform.translation
    diff = transform.x - map_transform.x, transform.y - map_transform.y
    print map_transform.x, map_transform.y
    print transform.x, transform.y

    rotation = pose.transform.rotation.z
    map_rotation = map_pose.transform.rotation.z
    diff_rotation = math.degrees(rotation - map_rotation)

    # costmap, path = generate_path(map_grid, diff_rotation, diff)
    # costmap_debug.publish(bridge.cv2_to_imgmsg(costmap, 'mono8'))
    # if path is None:
    #     path = []
    # path_debug.publish(
    #     Path(header=Header(frame_id='map'),
    #          poses=[PoseStamped(header=Header(frame_id='map'),
    #                             pose=Pose(position=Point(x=x_to_m(p[0]),
    #                                                      y=y_to_m(p[1]))))
    #                 for p in path]))

    # calculate theta_dot based on the current state
    if state['state'] == LINE_FOLLOWING:
        offset = 2
        goal = Vec2d(0, 1)  # always drive forward
        rotation = calculate_line_angle(camera)  # rotate to follow lines
        if abs(rotation) < 10:
            rotation = 0

        rotation /= 4.0

    else:
        # FIXME
        goal = calculate_gps_heading(gps, WAYPOINTS[state['tracking']])  # track the waypoint

        rotation = to180(goal.angle)
        goal = goal.with_angle(0)  # don't need to crab for GPS waypoint, steering will handle that
        # state_debug.publish(str(goal))

    # calculate translation based on obstacles

    potential = compute_potential(diff, map_grid, goal)
    translation = to180(potential.angle)

    rospy.loginfo('translation = %s, rotation = %s, speed = %s', translation, rotation, state['speed'])

    # don't rotate if bender needs to translate away from a line
    if state['state'] == LINE_FOLLOWING:
        translation_threshhold = 60
        rotation_throttle = 0
        if np.absolute(translation) > translation_threshhold:
            rotation = rotation * rotation_throttle
    rolling_average.append((translation, rotation, state['state']))
    update_drivetrain(translation, rotation, state['speed'])

    # rviz debug
    q = quaternion_from_euler(0, 0, math.radians(translation))
    heading_debug.publish(PoseStamped(header=Header(frame_id='map'),
                                      pose=Pose(position=Point(x=diff[0], y=diff[1]),
                                                orientation=Quaternion(q[0], q[1], q[2], q[3]))))


def main():
    global control, debug

    rospy.init_node(GUIDANCE_NODE)
    control = rospy.Publisher('input_vectors', Vector3, queue_size=3)
    debug = rospy.Publisher('debug', String, queue_size=3)

    # we randomly seem to get garbage messages that are only partially unpickled
    # ignore them until we can figure out what's going on
    def valid_message(msg):
        return not isinstance(msg['camera'], basestring)

    nav = rx_subscribe(topics.NAV).filter(valid_message)
    gps = nav.map(lambda msg: msg['gps']).buffer_with_count(GPS_BUFFER, 1)

    # recompute state whenever nav or gps emits
    state = nav.combine_latest(gps, lambda n, g: (n, g)) \
        .scan(compute_next_state, seed=DEFAULT_STATE)

    # update controls whenever position or state emits
    tf = rx_subscribe('/tf', TFMessage, parse=None, buffer_size=100)

    # only update map at 1hz to help with noise
    map_with_pos = rx_subscribe(topics.MAP, String)

    # cap update rate to 10Hz, otherwise guidance will fall behind
    pos = tf.let(extract_tf(topics.ODOMETRY_FRAME))

    pos.combine_latest(state, nav, map_with_pos, lambda o, s, n, m: (n, m.map_bytes, m.transform, o, s)) \
        .throttle_last(250) \
        .subscribe(update_control)

    rospy.spin()


if __name__ == '__main__':
    main()

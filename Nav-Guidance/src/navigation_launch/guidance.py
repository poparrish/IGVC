#!/usr/bin/env python
import pickle

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from rx import Observable
from std_msgs.msg import String

from gps import GPS_NODE
from guidance import calculate_potential, ATTRACTOR_THRESHOLD_MM, calculate_line_angle
from guidance.gps_guidance import dist_to_waypoint, calculate_gps_heading
from guidance.potential_field import partition
from nav import rx_subscribe, NAV_NODE
from util import Vec2d, avg, to180

GUIDANCE_NODE = "GUIDANCE"
GUIDANCE_HZ = 10

#
# Drivetrain
#

INITIAL_SPEED = 0.5  # gotta go FAST
INITIAL_DISTANCE_CUTOFF = 5000  # slow down once we're within 5m of something

NORMAL_SPEED = .5  # forward speed in m/s
MAX_ROTATION = 30  # max crabbing angle
MAX_TRANSLATION = 90  # max crabbing angle

control = None


def update_drivetrain(translation, rotation, speed):
    translation = max(-MAX_TRANSLATION, min(translation, MAX_TRANSLATION))
    rotation = max(-MAX_ROTATION, min(rotation, MAX_ROTATION))
    control.publish(np.array([translation, rotation, speed], dtype=np.float32))


def calculate_translation(lidar, camera, goal):
    """calculates the translation angle based on the potential field"""
    desired_heading = calculate_potential(lidar, camera, goal)

    translation = to180(desired_heading.angle)
    return translation


#
# Waypoints
#

GPS_BUFFER = 5  # buffer GPS messages to help with accuracy

FIRST_WAYPOINT_TOLERANCE = 1  # when to start tracking the first waypoint
WAYPOINT_TOLERANCE = .5  # precision in meters

#WAYPOINTS = [
#    (43.600314, -116.197164),  # N/W corner of field
#    (43.600248, -116.196955),  # center of field
#    (43.600314, -116.197164),  # N/W corner of field
#]

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
state_debug = rospy.Publisher('state', String, queue_size=3)


def compute_next_state(state, (nav, gps_buffer)):
    """guidance state machine"""
    state_debug.publish(state['state'])
    # check if we need to slow down
    speed = state['speed']
    if speed == INITIAL_SPEED:
        clusters = partition(nav['lidar'], cluster_mm=500)
        if len(clusters) > 0:
            closest = min([c for c in clusters], key=lambda v: v.mag)
        else:
            closest = 100000000
        if closest < INITIAL_DISTANCE_CUTOFF:
            speed = NORMAL_SPEED
            state['speed'] = speed

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


def update_control((msg, state)):
    """figures out what we need to do based on the current state and map"""
    camera = msg['camera']
    lidar = msg['lidar']
    # for v in lidar:
    #     if v.angle > 345 or v.angle < 15:
    #         print(v)
    gps = msg['gps']

    # calculate theta_dot based on the current state
    if state['state'] == LINE_FOLLOWING:
        goal = Vec2d(0, ATTRACTOR_THRESHOLD_MM)  # always drive forward
        rotation = calculate_line_angle(camera)  # rotate to follow lines
        if abs(rotation) < 10:
            rotation = 0

        rotation /= 4.0

    else:
        goal = calculate_gps_heading(gps, WAYPOINTS[state['tracking']])  # track the waypoint
        
        rotation = to180(goal.angle)
        goal = goal.with_angle(0)  # don't need to crab for GPS waypoint, steering will handle that
        # state_debug.publish(str(goal))

    # calculate translation based on obstacles
    potential = calculate_potential(lidar, camera, goal)
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

    # debug.publish(pickle.dumps({
    #     'camera': camera,
    #     'lidar': lidar,
    #     'gps': gps,
    #     'state': state,
    #     'goal': goal,
    #     'translation': translation,
    #     'rotation': rotation
    # }))


def main():
    global control, debug

    rospy.init_node(GUIDANCE_NODE)
    control = rospy.Publisher('control', numpy_msg(Floats), queue_size=3)
    debug = rospy.Publisher('debug', String, queue_size=3)

    # we randomly seem to get garbage messages that are only partially unpickled
    # ignore them until we can figure out what's going on
    def valid_message(msg):
        return not isinstance(msg['camera'], basestring)

    nav = rx_subscribe(NAV_NODE).filter(valid_message)
    gps = rx_subscribe(GPS_NODE).buffer_with_count(GPS_BUFFER, 1)

    # recompute state whenever nav or gps emits
    state = Observable.combine_latest(nav, gps, lambda n, g: (n, g)) \
        .scan(compute_next_state, seed=DEFAULT_STATE)

    # update controls whenever nav or state emits
    Observable.combine_latest(nav, state, lambda n, g: (n, g)) \
        .subscribe(update_control)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        rate.sleep()
    # rospy.spin()


if __name__ == '__main__':
    main()

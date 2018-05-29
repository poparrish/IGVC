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
from nav import rx_subscribe, NAV_NODE
from util import Vec2d
from util.vec2d import to180

GUIDANCE_NODE = "GUIDANCE"
GUIDANCE_HZ = 10 

#
# Drivetrain
#

DEFAULT_SPEED = 0.5  # forward speed in m/s
MAX_ROTATION = 90  # max crabbing angle
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

GPS_BUFFER = 20  # buffer GPS messages to help with accuracy

FIRST_WAYPOINT_TOLERANCE = 10  # when to start tracking the first waypoint

WAYPOINT_TOLERANCE = 1  # precision in meters
WAYPOINT_CONFIDENCE = 0.5  # percent of readings required to be within tolerance
WAYPOINTS = [
    (0,0),
    #(43.600189, -116.196871),  # N/W corner of field
    #(43.600313, -116.197169),  # S/E corner of field
    #(43.600258, -116.196969),  # Middle of field
]


def reached_waypoint(waypoint, gps_buffer, tolerance=WAYPOINT_TOLERANCE):
    waypoint = WAYPOINTS[waypoint]
    distances = [dist_to_waypoint(loc, waypoint) for loc in gps_buffer]

    within_tolerance = sum(1 for d in distances if d < tolerance)
    return within_tolerance / float(len(distances)) > WAYPOINT_CONFIDENCE


#
# State machine
#

LINE_FOLLOWING = 'LINE_FOLLOWING'
WAYPOINT_TRACKING = 'WAYPOINT_TRACKING'

DEFAULT_STATE = {
    'state': LINE_FOLLOWING,
    'tracking': 0
}

debug = None


def compute_next_state(state, gps_buffer):
    """guidance state machine"""

    if state['state'] == LINE_FOLLOWING:

        # if we're within range of the first waypoint, start tracking it
        if reached_waypoint(0, gps_buffer, tolerance=FIRST_WAYPOINT_TOLERANCE):
            rospy.loginfo('Begin tracking first waypoint')
            return {
                'state': WAYPOINT_TRACKING,
                'tracking': 0
            }

    if state['state'] == WAYPOINT_TRACKING:
        tracking = state['tracking']

        # if we've reached the current waypoint, start tracking the next one
        if reached_waypoint(tracking, gps_buffer):

            # ... unless we are at the last one, in which case we should resume normal navigation
            if tracking == len(WAYPOINTS) - 1:
                rospy.loginfo('Reached all waypoints, resuming normal operation')
                return {
                    'state': LINE_FOLLOWING
                }

            next = tracking + 1
            rospy.loginfo('Begin tracking waypoint %s', next)
            return {
                'state': WAYPOINT_TRACKING,
                'tracking': next
            }

    return state


def update_control((msg, state)):
    """figures out what we need to do based on the current state and map"""
    camera = msg['camera']
    lidar = msg['lidar']
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

    # calculate translation based on obstacles
    potential = calculate_potential(lidar, camera, goal)
    translation = to180(potential.angle)

    rospy.loginfo('translation = %s, rotation = %s', translation, rotation)
    update_drivetrain(translation, rotation, DEFAULT_SPEED)

    debug.publish(pickle.dumps({
        'camera': camera,
        'lidar': lidar,
        'gps': gps,
        'state': state,
        'goal': goal,
        'translation': translation,
        'rotation': rotation
    }))


def main():
    global control, debug

    rospy.init_node(GUIDANCE_NODE)
    control = rospy.Publisher('control', numpy_msg(Floats), queue_size=3)
    debug = rospy.Publisher('debug', String, queue_size=3)

    # navigation state machine
    state = rx_subscribe(GPS_NODE) \
        .buffer_with_count(GPS_BUFFER, 1) \
        .scan(compute_next_state, seed=DEFAULT_STATE)

    # we randomly seem to get garbage messages that are only partially unpickled
    # ignore them until we can figure out what's going on
    def valid_message(msg):
        return not isinstance(msg['camera'], basestring)

    # update controls whenever the nav or state changes
    nav = rx_subscribe(NAV_NODE).filter(valid_message)
    Observable.combine_latest(nav, state, lambda n, s: (n, s)) \
        .throttle_first(1000.0 / GUIDANCE_HZ) \
        .subscribe(update_control)

    rospy.spin()


if __name__ == '__main__':
    main()

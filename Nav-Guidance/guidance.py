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
GUIDANCE_HZ = 5

#
# Drivetrain
#

DEFAULT_SPEED = 0  # forward speed in m/s
MAX_TRANSLATION = 90  # max crabbing angle

control = None


def update_drivetrain(translation, rotation, speed):
    translation = max(-MAX_TRANSLATION, min(translation, MAX_TRANSLATION))
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
WAYPOINT_CONFIDENCE = 0.5  # number of readings required to be within tolerance
WAYPOINTS = [
    (43.600189, -116.196871),  # N/W corner of field
    (43.600313, -116.197169),  # S/E corner of field
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
    'state': WAYPOINT_TRACKING,
    'tracking': 0
}

debug = None


def compute_next_state(state, gps_buffer):
    """guidance state machine"""

    if state['state'] == LINE_FOLLOWING:

        # if we're within range of the first waypoint, start tracking it
        if reached_waypoint(WAYPOINTS[0], gps_buffer, tolerance=FIRST_WAYPOINT_TOLERANCE):
            rospy.loginfo('Begin tracking waypoint %s', 0)
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
                rospy.loginfo('Reached all waypoints, resuming normal operation', 0)
                return {
                    'state': LINE_FOLLOWING
                }

            rospy.loginfo('Begin tracking waypoint %s', 0)
            return {
                'state': WAYPOINT_TRACKING,
                'tracking': tracking + 1
            }

    return state


def update_control(msg, state):
    """figures out what we need to do based on the current state map"""
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

    debug.publish({
        'camera': camera,
        'lidar': lidar,
        'gps': gps,
        'state': state,
        'translation': translation,
        'rotation': rotation
    })


def main():
    global control, debug

    rospy.init_node(GUIDANCE_NODE)
    control = rospy.Publisher('control', numpy_msg(Floats), queue_size=3)
    debug = rospy.Publisher('debug', String, queue_size=3)

    nav = rx_subscribe(NAV_NODE)
    gps = rx_subscribe(GPS_NODE).buffer_with_count(GPS_BUFFER, 1)

    # navigation state machine
    state = gps.scan(compute_next_state, seed=DEFAULT_STATE)

    # update the control node on change
    Observable.combine_latest(nav, state, update_control) \
        .throttle_first(1000.0 / GUIDANCE_HZ) \
        .subscribe()

    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python
import math
import traceback

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, TransformStamped, Transform, Point32
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String, Header, Int16
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage

import topics
from guidance import compute_potential
from guidance.attractor_placement import generate_path
from guidance.gps_guidance import dist_to_waypoint, calculate_gps_heading
from guidance.potential_field import extract_repulsors, ATTRACTOR_THRESHOLD_MM
from mapping import MAP_SIZE_PIXELS, MAP_SIZE_METERS
from util import Vec2d, avg, to180
from util.rosutil import extract_tf, rx_subscribe

GUIDANCE_NODE = "GUIDANCE"
GUIDANCE_HZ = 10

#
# Drivetrain
#

INITIAL_SPEED = 1  # gotta go FAST
INITIAL_DISTANCE_CUTOFF = 5000  # slow down once we're within 5m of something

NORMAL_SPEED = .5  # forward speed in m/s
MAX_ROTATION = 90  # max crabbing angle
MAX_TRANSLATION = 90  # max crabbing angle

control = None


def update_drivetrain(translation, rotation, speed):
    translation = max(-MAX_TRANSLATION, min(translation, MAX_TRANSLATION))
    rotation = max(-MAX_ROTATION, min(rotation, MAX_ROTATION))
    # speed, velocity_vector, theta_dot
    control.publish(Vector3(x=speed, y=translation, z=-rotation))


#
# Waypoints
#

GPS_BUFFER = 5  # buffer GPS messages to help with accuracy

FIRST_WAYPOINT_TOLERANCE = 1  # when to start tracking the first waypoint
WAYPOINT_TOLERANCE = .5  # precision in meters

WAYPOINTS = [
    # (43.600314, -116.197164),  # N/W corner of field
    (43.600248, -116.196955),  # center of field
    (43.600314, -116.197164),  # N/W corner of field
]

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

# WAYPOINTS = [
#     endof,
#     qual1,
#     qual2,
#     qual1
# ]

northwp = [42.6790984, -83.1949250546]
midwp = [42.6789603912, -83.1951132036]
southwp = [42.6788026, -83.1949093082]


# WAYPOINTS = [
#     northwp,
#     midwp,
#     southwp
# ]


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

heading_debug = rospy.Publisher(topics.POTENTIAL_FIELD, PoseStamped, queue_size=1)
path_debug = rospy.Publisher('guidance/path', Path, queue_size=1)
obstacle_debug = rospy.Publisher('guidance/obstacles', PointCloud, queue_size=1)


def compute_next_state(state, gps_buffer):
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


def update_control((gps, costmap, pose, line_angle, state)):
    """figures out what we need to do based on the current state and map"""
    map_pose = costmap.transform

    transform = pose.transform.translation
    map_transform = map_pose.transform.translation
    diff = transform.x - map_transform.x, transform.y - map_transform.y
    print map_transform.x, map_transform.y
    print transform.x, transform.y

    rotation = pose.transform.rotation.z
    map_rotation = map_pose.transform.rotation.z
    diff_rotation = math.degrees(rotation - map_rotation)

    path = generate_path(costmap.costmap_bytes, diff_rotation, diff)

    if path is None:
        path = []
    path_debug.publish(
        Path(header=Header(frame_id='map'),
             poses=[PoseStamped(header=Header(frame_id='map'),
                                pose=Pose(position=Point(x=x_to_m(p[0]),
                                                         y=y_to_m(p[1]))))
                    for p in path]))

    # calculate theta_dot based on the current state
    if state['state'] == LINE_FOLLOWING:
        offset = 5
        if len(path) < offset + 1:
            goal = Vec2d(0, ATTRACTOR_THRESHOLD_MM)  # always drive forward
        else:
            point = path[offset]
            goal = Vec2d.from_point(x_to_m(point[0] + 0.5), y_to_m(point[1] + 0.5))
            goal = goal.with_magnitude(ATTRACTOR_THRESHOLD_MM)
        rotation = line_angle.data  # rotate to follow lines
        if abs(rotation) < 10:
            rotation = 0

        rotation /= 1.0

    else:
        # FIXME
        goal = calculate_gps_heading(gps, WAYPOINTS[state['tracking']])  # track the waypoint

        rotation = to180(goal.angle)
        goal = goal.with_angle(0)  # don't need to crab for GPS waypoint, steering will handle that
        # state_debug.publish(str(goal))

    # calculate translation based on obstacles
    repulsors = extract_repulsors(diff, costmap.map_bytes)
    potential = compute_potential(repulsors, goal)
    obstacle_debug.publish(PointCloud(header=Header(frame_id=topics.ODOMETRY_FRAME),
                                      points=[Point32(x=v.x / 1000.0, y=v.y / 1000.0) for v in repulsors]))
    translation = to180(potential.angle)

    rospy.loginfo('translation = %s, rotation = %s, speed = %s', translation, rotation, state['speed'])

    # don't rotate if bender needs to translate away from a line
    # if state['state'] == LINE_FOLLOWING:
    #     translation_threshhold = 60
    #     rotation_throttle = 0
    #     if np.absolute(translation) > translation_threshhold:
    #         rotation = rotation * rotation_throttle
    rolling_average.append((translation, rotation, state['state']))
    update_drivetrain(translation, rotation, state['speed'])

    # rviz debug
    q = quaternion_from_euler(0, 0, math.radians(translation))
    heading_debug.publish(PoseStamped(header=Header(frame_id='map'),
                                      pose=Pose(position=Point(x=diff[0], y=diff[1]),
                                                orientation=Quaternion(q[0], q[1], q[2], q[3]))))


def main():
    global control

    rospy.init_node(GUIDANCE_NODE)
    control = rospy.Publisher('input_vectors', Vector3, queue_size=3)

    gps = rx_subscribe(topics.GPS)

    # compute state based on GPS coordinates
    state = gps \
        .buffer_with_count(GPS_BUFFER, 1) \
        .scan(compute_next_state, seed=DEFAULT_STATE)

    # update controls whenever position or state emits
    tf = rx_subscribe('/tf', TFMessage, parse=None, buffer_size=100)

    costmap = rx_subscribe(topics.COSTMAP, String)

    line_angle = rx_subscribe(topics.LINE_ANGLE, Int16, parse=None).start_with(Int16(0))
    pos = tf.let(extract_tf(topics.ODOMETRY_FRAME)).start_with(
        TransformStamped(transform=Transform(rotation=Quaternion(0, 0, 0, 1))))

    pos.combine_latest(state, gps, costmap, line_angle,
                       lambda o, s, g, m, a: (g, m, o, a, s)) \
        .throttle_last(100) \
        .subscribe(on_next=update_control,
                   on_error=lambda e: rospy.logerr(traceback.format_exc(e)))

    rospy.spin()


if __name__ == '__main__':
    main()

import math

from gpxpy.geo import haversine_distance

from util import Vec2d


# https://www.movable-type.co.uk/scripts/latlong.html
def initial_bearing((lat1, lon1), (lat2, lon2)):
    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - \
        math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


def dist_to_waypoint(loc, (wlat, wlon)):
    """returns the distance to a GPS waypoint (in meters)"""
    return haversine_distance(loc['lat'], loc['lon'], wlat, wlon)


def calculate_gps_heading(loc, waypoint):
    """returns a vector, where the angle is the initial bearing and the magnitude
    is the distance between two GPS coordinates"""

    dist = dist_to_waypoint(loc, waypoint)
    angle = initial_bearing((loc['lat'], loc['lon']), waypoint) - loc['gps_heading']

    return Vec2d(angle, dist)

def current_angle(loc):
    """just returns the current angle from pixhawk"""
    return loc['pitch']

def current_roll(loc):
    """just returns the current roll from pixhawk"""
    return loc['roll']
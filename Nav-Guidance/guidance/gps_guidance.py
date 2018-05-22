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


def calculate_gps_heading(loc, dest):
    """returns a vector with the initial bearing and distance between two points"""

    dist = haversine_distance(loc.lat, loc.lon, dest[0], dest[1])
    angle = initial_bearing((loc.lat, loc.lon), dest) - loc.heading

    return Vec2d(angle, dist)

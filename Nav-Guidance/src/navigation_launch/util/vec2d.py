import math


def to180(angle):
    angle %= 360
    if angle > 180:
        angle = -360 + angle
    return angle


def clamp360(angle):
    angle %= 360
    if angle < 0:
        angle += 360
    return angle


def avg(coll):
    return sum(coll) / len(coll)


class Vec2d:
    """Immutable 2d vector. Overloads most common operators."""

    def __init__(self, angle, mag):
        self.angle = clamp360(angle)
        self.mag = mag

    @property
    def x(self):
        return math.cos(math.radians(self.angle)) * self.mag

    @property
    def y(self):
        return math.sin(math.radians(self.angle)) * self.mag

    @staticmethod
    def from_point(x, y):
        angle = math.degrees(math.atan2(y, x))
        return Vec2d(angle, math.sqrt(x ** 2 + y ** 2))

    def with_angle(self, angle):
        return Vec2d(clamp360(angle), self.mag)

    def with_magnitude(self, mag):
        if mag < 0:
            return Vec2d.from_point(-self.x, -self.y).with_magnitude(-mag)
        return Vec2d(self.angle, mag)

    def __add__(self, other):
        return Vec2d.from_point(self.x + other.x, self.y + other.y)

    def __radd__(self, other):
        return Vec2d.from_point(self.x + other, self.y + other)

    def __sub__(self, other):
        return Vec2d.from_point(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return self.with_magnitude(self.mag * scalar)

    def __div__(self, scalar):
        return self.with_magnitude(self.mag / scalar)

    def __neg__(self):
        return self * -1

    def __str__(self):
        return "(%s, %s)" % (self.angle, self.mag)

    def __repr__(self):
        return self.__str__()

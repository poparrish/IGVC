#include "Vec2d.h"
#include "math.h"

Vec2d::Vec2d(float x, float y) : x(x), y(y) {}

float Vec2d::angle() const {
    return atan2(y, x);
}

float Vec2d::magnitude() const {
    return sqrt(x * x + y * y);
}

Vec2d Vec2d::operator*(float scalar) const {
    return Vec2d(x * scalar, y * scalar);
}

float Vec2d::operator*(Vec2d const &other) const {
    return x * other.x + y * other.y;
}

std::ostream &operator<<(std::ostream &os, const Vec2d &d) {
    os << "x: " << d.x << " y: " << d.y;
    return os;
}

float Vec2d::distance(const Vec2d &other) const {
    float dx = x - other.x;
    float dy = y * other.y;
    return sqrt(dx * dx + dy * dy);
}

Vec2d Vec2d::operator-(Vec2d const &other) const {
    return Vec2d(x - other.x, y - other.y);
}

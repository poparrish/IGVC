#include <cfloat>
#include "Vec2d.h"
#include "math.h"

float Vec2d::angle() const {
    return atan2f(y, x);
}

float Vec2d::magnitude() const {
    return sqrtf(x * x + y * y);
}

Vec2d Vec2d::operator*(float scalar) const {
    return {x * scalar, y * scalar};
}

float Vec2d::operator*(Vec2d const &other) const {
    return x * other.x + y * other.y;
}

std::ostream &operator<<(std::ostream &os, const Vec2d &d) {
    os << "x: " << d.x << ", y: " << d.y;
    return os;
}

float Vec2d::distance(const Vec2d &other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return sqrtf(dx * dx + dy * dy);
}

Vec2d Vec2d::operator-(Vec2d const &other) const {
    return {x - other.x, y - other.y};
}

Vec2d Vec2d::abs() const {
    return {fabsf(x), fabsf(y)};
}

float Vec2d::angle(const Vec2d &other) const {
    return acosf(*this * other / (this->magnitude() * other.magnitude()));
}

Vec2d Vec2d::from_angle(float rad, float length) {
    float x = cosf(rad) * length;
    float y = sinf(rad) * length;
    return {x, y};
}

bool Vec2d::operator==(Vec2d const &other) const {
    return fabsf(x - other.x) < FLT_EPSILON &&
           fabsf(y - other.y) < FLT_EPSILON;
}


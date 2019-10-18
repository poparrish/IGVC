#ifndef SLAM_VEC2D_H
#define SLAM_VEC2D_H

#include <string>
#include <ostream>

class Vec2d {
public:
    static Vec2d from_angle(float rad, float length);

    float x, y;

    Vec2d() = default;

    Vec2d(float x, float y) : x{x}, y{y} {};

    float angle() const;

    float magnitude() const;

    Vec2d abs() const;

    /// multiply by a scalar
    Vec2d operator*(float scalar) const;

    /// dot product
    float operator*(Vec2d const &other) const;

    Vec2d operator-(Vec2d const &other) const;

    bool operator==(Vec2d const &other) const;

    float distance(const Vec2d &other) const;

    float angle(const Vec2d &other) const;

    friend std::ostream &operator<<(std::ostream &os, const Vec2d &d);
};

#endif //SLAM_VEC2D_H

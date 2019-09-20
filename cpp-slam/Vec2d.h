#ifndef SLAM_VEC2D_H
#define SLAM_VEC2D_H

#include <string>
#include <ostream>

class Vec2d {
public:
    float x, y; // TODO const

    Vec2d() = default;

    Vec2d(float x, float y);

    float angle() const;

    float magnitude() const;

    /// multiply by a scalar
    Vec2d operator*(float scalar) const;

    /// dot product
    float operator*(Vec2d const &other) const;

    Vec2d operator-(Vec2d const &other) const;

    float distance(const Vec2d &other) const;

    friend std::ostream &operator<<(std::ostream &os, const Vec2d &d);
};

#endif //SLAM_VEC2D_H

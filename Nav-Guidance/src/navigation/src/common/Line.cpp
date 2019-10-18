#include <cmath>
#include <iostream>
#include "Line.h"

float Line::distance(const Vec2d &point) const {
    // see http://paulbourke.net/geometry/pointlineplane/
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float length_squared = (dx * dx) + (dy * dy);

    float u = ((point.x - start.x) * dx +
               (point.y - start.y) * dy) / (length_squared);
    u = fmaxf(0, fminf(1, u));

    Vec2d intersect{start.x + u * dx,
                    start.y + u * dy};
    return intersect.distance(point);
}

float Line::angle(const Vec2d &point) const {
    return end.angle(point);
}


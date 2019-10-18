#ifndef SLAM_LINE_H
#define SLAM_LINE_H

#include "Vec2d.h"

class Line {
public:
    Vec2d start, end;

    Line(const Vec2d &start, const Vec2d &end) : start(start), end(end) {}

    float distance(const Vec2d &point) const;

    float angle(const Vec2d &point) const;
};


#endif //SLAM_LINE_H

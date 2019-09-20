#ifndef SLAM_FEATURE_EXTRACTION_H
#define SLAM_FEATURE_EXTRACTION_H

#include <vector>

using std::vector;

struct Circle {
    Vec2d center;
    float radius;
    float stdev;
};

vector<Circle> extract_circles(const vector<Vec2d> &points);

#endif //SLAM_FEATURE_EXTRACTION_H

#ifndef SLAM_FEATURE_EXTRACTION_H
#define SLAM_FEATURE_EXTRACTION_H

#include <vector>
#include <memory>
#include "../common/Line.h"
#include "../common/math_util.h"

using std::vector;

struct Circle {
    Vec2d center;
    float radius;
    float stdev;
};

struct Features {
    vector<Circle> circles;
    vector<Line> lines;
    vector<vector<Vec2d>> segments;
};

struct LineFitConfig {
    float maxError;
    int minPoints;
};

struct ArcConfig {
    float minMean;
    float maxMean;
    float maxStdev;
};

struct SegmentConfig {
    float maxDist;
    float minSegmentAperture;
};

struct FeatureConfig {
    LineFitConfig lines;
    ArcConfig arcs;
    SegmentConfig segments;
};

Features extract_features(const vector<Vec2d> &points, const FeatureConfig &config);

#endif //SLAM_FEATURE_EXTRACTION_H

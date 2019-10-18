#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "feature_extraction.h"
#include "../common/Vec2d.h"
#include "../common/math_util.h"

using std::vector;
using std::shared_ptr;

float aperture(const Vec2d &l, const Vec2d &m, const Vec2d &r) {
    auto lm = m - l;
    auto mr = r - m;

    return lm.angle(mr);
}

vector<vector<Vec2d>> extract_segments(const vector<Vec2d> &points, float threshold) {
    vector<vector<Vec2d>> segments;
    if (points.empty()) {
        return segments;
    }

    auto last = points[0];
    auto seg = vector<Vec2d>();
    seg.push_back(last);

    for (auto i = 1; i < points.size(); ++i) {
        auto curr = points[i];

        if (last.distance(curr) > threshold) {
            segments.emplace_back(seg);
            seg = vector<Vec2d>();
        }

        seg.push_back(curr);
        last = curr;
    }
    segments.push_back(seg);
    return segments;
}

vector<vector<Vec2d>> split_segments(const vector<vector<Vec2d>> &segments, float minAngle) {
    vector<vector<Vec2d>> split;

    for (const auto &points: segments) {
        if (points.empty()) {
            continue;
        }

        auto seg = vector<Vec2d>();
        seg.push_back(points[0]);

        for (auto i = 1; i < points.size() - 1; ++i) {
            auto &left = points[i - 1];
            auto &mid = points[i];
            auto &right = points[i + 1];
            seg.push_back(mid);

            if (aperture(left, mid, right) < minAngle) {
                split.emplace_back(seg);
                seg = vector<Vec2d>();
            }
        }
    }

    return segments;
}

float slope(const Vec2d &left, const Vec2d &mid) {
    return (mid.y - left.y) / (mid.x - left.x);
}

Vec2d find_center(const Vec2d &left, const Vec2d &mid, const Vec2d &right) {
    auto ma = slope(left, mid);
    auto mb = slope(mid, right);

    auto x = (ma * mb * (left.y - right.y) + mb * (left.x + mid.x) - ma * (mid.x + right.x)) / (2 * (mb - ma));
    auto y = -1 / ma * (x - (left.x + mid.x) / 2) + (left.y + mid.y) / 2;

    if (y == INFINITY) {
        y = -1 / mb * (x - (mid.x + right.x) / 2) + (mid.y + right.y) / 2;
    }

    return {x, y};
}

/** Fast check to see if a segment is potentially a circle */
bool is_circle_candidate(const vector<Vec2d> &segment, Vec2d &center, float &diameter) {
    if (segment.size() <= 3) {
        return false;
    }

    auto left = segment.front();
    auto right = segment.back();
    auto middle = segment[segment.size() / 2];

    center = find_center(left, middle, right);

    auto angle = atanf((left.x - right.x) / (left.y - right.y));
    diameter = left.distance(right);
    auto point = -fabs(cos(angle) - sin(angle));

    auto min = -0.7 * diameter, max = -0.1 * diameter;
    if (min <= point && point <= max) {
        ROS_INFO_STREAM("Found circle candidate at " << center << " diameter " << diameter);
        return true;
    }
    ROS_INFO("Not a circle candidate, point %f (diameter %f) not between [%f %f]", point, diameter, min, max);
    return true;
}

/**
 * More thorough check to see if a segment forms a circle
 * @param segment
 * @param max_stdev the max stdev in aperture (radians)
 * @param meanStart min aperture mean angle (degrees)
 * @param meanEnd max aperture mean angle (degrees)
 * @return
 */
bool fits_circle_constraints(const vector<Vec2d> &segment,
                             float &stdev,
                             const ArcConfig &config
) {
    auto left = segment.front();
    auto right = segment.back();

    auto apertures = vector<float>(segment.size() - 2);
    for (int i = 1; i < segment.size() - 1; ++i) {
        apertures[i - 1] = aperture(left, segment[i], right);
    }

    auto mean = math::mean(apertures);
    if (config.minMean >= mean || mean >= config.maxMean) {
        ROS_INFO("Not a circle, mean %f not between [%f %f]", mean, config.minMean, config.maxMean);
        return false;
    }

    stdev = math::stdev(apertures, mean);
    if (stdev > config.maxStdev) {
        ROS_INFO("Not a circle, stdev %f greater than %f", stdev, config.maxStdev);
        return false;
    }

    return true;
}


void recursive_fit_lines(const vector<Vec2d> &segment, int start, int end, const LineFitConfig &threshold,
                         vector<Line> &lines) {
    if (end <= start || end - start < std::max(3, threshold.minPoints)) {
        return;
    }

    const Vec2d &s = segment[start];
    const Vec2d &e = segment[end];
    Line line(s, e);

    float maxError = 0;
    int maxSegment = 0;

    for (int i = start + 1; i < end - 1; ++i) {
        const Vec2d &d = segment[i];
        float dist = line.distance(d);

        if (dist > maxError) {
            maxError = dist;
            maxSegment = i;
        }
    }

    // if the error is too large, split the segment
    if (maxError > threshold.maxError) {
        recursive_fit_lines(segment, start, maxSegment, threshold, lines);
        recursive_fit_lines(segment, maxSegment, end, threshold, lines);
    } else {
        lines.push_back(line);
    }
}

Features extract_features(const vector<Vec2d> &points, const FeatureConfig &config) {
    Features features;
    const auto segments = split_segments(extract_segments(points, config.segments.maxDist), config.segments.minSegmentAperture);
    ROS_INFO("Segment count: %zu", segments.size());

    for (const auto &segment : segments) {
        Vec2d center{0, 0};
        float diameter, stdev;
        if (is_circle_candidate(segment, center, diameter)
            && fits_circle_constraints(segment, stdev, config.arcs)) {
            Circle c = {.center=center, .radius=diameter / 2, .stdev=stdev};
            features.circles.push_back(c);
        } else {
            recursive_fit_lines(segment, 0, segment.size() - 1, config.lines, features.lines);
        }
    }

    features.segments = segments;
    return features;
}


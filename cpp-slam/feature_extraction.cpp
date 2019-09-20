#include <vector>
#include "Vec2d.h"
#include "feature_extraction.h"
#include <cmath>

using std::vector;

vector<vector<Vec2d> *> extract_segments(const vector<Vec2d> &points, float threshold) {
    vector<vector<Vec2d> *> segments;
    if (points.empty()) {
        return segments;
    }

    auto last = points[0];
    auto *seg = new vector<Vec2d>();
    seg->push_back(last);

    for (auto i = 1; i < points.size(); ++i) {
        auto curr = points[i];

        if (last.distance(curr) > threshold) {
            segments.push_back(seg);
            seg = new vector<Vec2d>();
        }

        seg->push_back(curr);
        last = curr;
    }
    segments.push_back(seg);
    return segments;
}

float mean(vector<float> &values) {
    if (values.empty()) {
        return 0;
    }
    float avg = 0;
    for (auto const &value:values) {
        avg += value;
    }
    return avg / values.size();
}

float stdev(vector<float> &values, float mean) {
    if (values.empty()) {
        return 0;
    }

    auto diffs = 0;
    for (auto const &value:values) {
        auto diff = value - mean;
        diffs += diff * diff;
    }

    return sqrt(1.0 / values.size() * diffs);
}

float aperture(const Vec2d &l, const Vec2d &m, const Vec2d &r) {
    auto lm = m - l;
    auto mr = r - m;
    return acos((lm * mr) / (lm.magnitude() * mr.magnitude()));
}

float slope(const Vec2d &left, const Vec2d &mid) {
    return (mid.y - left.y) / (mid.x - left.x);
}

Vec2d find_center(const Vec2d &left, const Vec2d &mid, const Vec2d &right) {
    auto ma = slope(left, mid);
    auto mb = slope(mid, right);

    auto x = (ma * mb * (left.y - right.y) - mb * (left.x + mid.x) - ma * (mid.x + right.x)) / (2 * (mb - ma));
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

    auto angle = atan((left.x - right.x) / (left.y - right.y));
    auto point = center.x * cos(angle) - center.y * sin(angle);
    diameter = left.distance(right);
    return -0.8 * diameter <= point && -point <= -0.1 * diameter;
}

/**
 * More thorough check to see if a segment forms a circle
 * @param segment
 * @param max_stdev the max stdev in aperture (radians) TODO: Why different units
 * @param meanStart min aperture mean angle (degrees)
 * @param meanEnd max aperture mean angle (degrees)
 * @return
 */
bool fits_circle_constraints(const vector<Vec2d> &segment,
                             float &st,
                             float max_stdev = 0.15,
                             float meanStart = 1.57,
                             float meanEnd = 2.37
) {
    auto left = segment.front();
    auto right = segment.back();

    auto apertures = vector<float>(segment.size() - 2);
    for (int i = 1; i < segment.size() - 1; ++i) {
        apertures[i - 1] = aperture(left, segment[i], right);
    }

    auto apertures_mean = mean(apertures);
    if (meanStart >= apertures_mean || apertures_mean >= meanEnd) {
        return false;
    }
    st = stdev(apertures, apertures_mean);
    return st <= max_stdev;
}

vector<Circle> extract_circles(const vector<Vec2d> &points) {
    vector<Circle> circles;

    auto segments = extract_segments(points, 10);
    for (const auto &segment : segments) {
        Vec2d center{};
        float diameter, stdev;
        if (is_circle_candidate(*segment, center, diameter)
            && fits_circle_constraints(*segment, stdev)) {
            Circle c = {.center=center, .radius=diameter / 2, .stdev=stdev};
            circles.push_back(c);
        }
    }

    return circles;
}
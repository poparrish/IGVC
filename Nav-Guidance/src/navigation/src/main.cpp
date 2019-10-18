#include <iostream>
#include <vector>
#include <cmath>
#include "common/Vec2d.h"
#include "mapping/feature_extraction.h"
#include "../../../extern/matplotlib-cpp/matplotlibcpp.h"
#include "common/math_util.h"

namespace plt = matplotlibcpp;

void drawCircle(float radius, const Vec2d &d) {
    int res = 100;
    vector<double> x(res), y(res);
    for (int i = 0; i < res; ++i) {
        double t = 2 * M_PI * i / (res - 1);
        x.at(i) = radius * cos(t) + d.x;
        y.at(i) = radius * sin(t) + d.y;
    }

    plt::plot(x, y, "");
}

void drawLine(const Line &points) {
    vector<float> x = {points.start.x, points.end.x};
    vector<float> y = {points.start.y, points.end.y};

    plt::plot(x, y, "");
}

void genCircle(std::vector<Vec2d> &points, float x, float y, float yScale) {
    const auto res = 20;
    for (int i = 0; i <= res; ++i) {
        points.emplace_back(x + cosf(PI * i / res), y + yScale * sin(PI * i / res));
    }
}

int main() {
    // Create test data
    std::vector<Vec2d> points;
    for (int i = 0; i < 7; ++i) {
        points.emplace_back(4, -i + 4);
    }
    for (int i = 0; i < 4; ++i) {
        points.emplace_back(i + 4, -3);
    }

    genCircle(points, -2, 3, -1);
    genCircle(points, 0, 3, -1);
    genCircle(points, 2, 3, -1);

    genCircle(points, -3, -3, 5);

    genCircle(points, -3, -5, 1);
    for (int i = 0; i < 5; ++i) {
        points.emplace_back(-2, i - 8);
    }

    const FeatureConfig config = {.lines={.maxError=.5, .minPoints=3}};
    const auto features = extract_features(points, config);
    for (const auto &circle : features.circles) {
        drawCircle(circle.radius, circle.center);
    }

    for (const auto &item : features.lines) {
        drawLine(item);
    }

    vector<float> x, y;
    for (const auto &point : points) {
        x.push_back(point.x);
        y.push_back(point.y);
    }
    plt::scatter(x, y);
    plt::show();

    return 0;
}

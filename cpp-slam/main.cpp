#include <iostream>
#include <vector>
#include "Vec2d.h"
#include "feature_extraction.h"
#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;

struct Point {
    int x;
    int y;
};

struct Landmark {
    std::vector<Point> points;
};

struct Map {
    std::vector<Landmark> landmarks;
};

int main() {
    // Create test data
    std::vector<Vec2d> points;
    points.emplace_back(2+-1, 0);
    points.emplace_back(2+-0.707, 0.707);
    points.emplace_back(2+0, 1);
    points.emplace_back(2+0.707, 0.707);
    points.emplace_back(2+1, 0);

    auto circles = extract_circles(points);
    for (const auto &circle : circles) {
        std::cout << "A circle" << std::endl;
        std::cout << circle.center << " " << circle.radius << std::endl;
    }

    return 0;
}
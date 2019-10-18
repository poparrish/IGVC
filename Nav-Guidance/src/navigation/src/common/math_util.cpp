#include <vector>
#include <cmath>
#include "math_util.h"

namespace math {
    float deg2rad(float deg) {
        return deg * PI / 180;
    }

    float rad2deg(float rad) {
        return rad * 180 / PI;
    }

    float mean(const std::vector<float> &values) {
        if (values.empty()) {
            return 0;
        }
        float avg = 0;
        for (auto const &value:values) {
            avg += value;
        }
        return avg / values.size();
    }

    float stdev(const std::vector<float> &values, float mean) {
        if (values.empty()) {
            return 0;
        }

        float diffs = 0;
        for (auto const &value:values) {
            auto diff = value - mean;
            diffs += diff * diff;
        }

        return sqrt(1.0 / values.size() * diffs);
    }
}

#ifndef NAVIGATION_MATH_UTIL_H
#define NAVIGATION_MATH_UTIL_H

#define PI 3.14159
namespace math {
    float deg2rad(float deg);

    float rad2deg(float rad);

    float mean(const std::vector<float> &values);

    float stdev(const std::vector<float> &values, float mean);
}

#endif //NAVIGATION_MATH_UTIL_H

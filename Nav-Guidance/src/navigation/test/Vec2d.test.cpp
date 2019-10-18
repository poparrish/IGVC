#include <catch_ros/catch.hpp>
#include <iomanip>
#include "../src/common/Vec2d.h"
#include "../src/common/math_util.h"

std::string format(const Vec2d &v) {
    std::ostringstream str;
    str << std::setprecision(4) << std::fixed << v.x << ", " << v.y;
    return str.str();
}

void checkAngle(float rad, float length, const Vec2d expected) {
    const Vec2d &actual = Vec2d::from_angle(rad, length);
    REQUIRE(format(actual) == format(expected));
}

TEST_CASE("Vec2d::from_angle", "[unit-test]") {
    checkAngle(0, 1, {1, 0});
    checkAngle(PI / 2, 1, {0, 1});
    checkAngle(PI, 1, {-1, 0});
}

#include <catch_ros/catch.hpp>
#include "../src/common/Line.h"

TEST_CASE("distance", "[unit-test]") {
    Line line({1, 1}, {1, 5});

    REQUIRE(line.distance({1, -1}) == 2);
    REQUIRE(line.distance({2, 1}) == 1);
    REQUIRE(line.distance({2, 5}) == 1);
    REQUIRE(line.distance({0, 1}) == 1);
    REQUIRE(line.distance({0, 5}) == 1);
}

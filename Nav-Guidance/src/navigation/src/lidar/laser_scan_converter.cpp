#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include "laser_scan_converter.h"
#include "../common/Vec2d.h"

std::vector<Vec2d> scanToVec2d(const sensor_msgs::LaserScan::ConstPtr &scan) {
    const unsigned long size = scan->intensities.size();
    std::vector<Vec2d> result;
    result.reserve(size);

    for (int i = 0; i < size; ++i) {
        float intensity = scan->intensities[i];
        float range = scan->ranges[i];
        if (range == INFINITY) {
            continue;
        }
        float angle = i * scan->angle_increment + scan->angle_min;
        const Vec2d &x = Vec2d::from_angle(angle, range);
        result.push_back(x);
    }

    return result;
}


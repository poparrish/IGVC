#ifndef NAVIGATION_TOPICS_H
#define NAVIGATION_TOPICS_H

#include "../common/Vec2d.h"

std::vector<Vec2d> scanToVec2d(const sensor_msgs::LaserScan::ConstPtr &scan);

#endif //NAVIGATION_TOPICS_H

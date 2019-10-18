#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "lidar/laser_scan_converter.cpp"
#include "mapping/feature_extraction.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "topics.h"

visualization_msgs::Marker createMarker() {
    visualization_msgs::Marker m;
    m.header.frame_id = "laser";
    m.header.stamp = ros::Time::now();
    m.ns = "feature";
    m.action = visualization_msgs::Marker::ADD;
    return m;
}

visualization_msgs::Marker createCircleMarker(const Circle &circle) {
    auto m = createMarker();
    m.type = visualization_msgs::Marker::CYLINDER;

    m.pose.position.x = circle.center.x;
    m.pose.position.y = circle.center.y;
    m.pose.position.z = 0;

    m.scale.x = circle.radius * 2;
    m.scale.y = circle.radius * 2;
    m.scale.z = 0.01;

    m.color.g = 1.0f;
    m.color.a = 0.3f;
    return m;
}

visualization_msgs::Marker createLineMarker(const std::vector<Vec2d> &segment) {
    auto m = createMarker();
    m.type = visualization_msgs::Marker::LINE_STRIP;
    for (const auto &point: segment) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        m.points.emplace_back(p);
    }
    m.scale.x = 0.01;
    m.color.g = 1.0f;
    m.color.a = 0.3f;
    return m;
}

class FeatureExtractionNode {
private:
    ros::Publisher markerDebug;
    FeatureConfig config;

public:
    explicit FeatureExtractionNode(const ros::Publisher &markerDebug, const FeatureConfig &config)
            : markerDebug(markerDebug), config(config) {}

    void scanReceived(const sensor_msgs::LaserScan::ConstPtr &scan) {
        ROS_INFO("SCAN RECEIVED");

        const auto vecs = scanToVec2d(scan);
        const auto features = extract_features(vecs, config);

        if (markerDebug) {
            int id = 0;
            visualization_msgs::MarkerArray markers;
            for (const auto &segment : features.segments) {
                auto marker = createLineMarker(segment);
                marker.id = id++;
                markers.markers.emplace_back(marker);
            }

            for (const auto &circle : features.circles) {
                auto marker = createCircleMarker(circle);
                marker.id = id++;
                markers.markers.emplace_back(marker);
            }

            markerDebug.publish(markers);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_extraction");
    ros::NodeHandle n;

    const auto markerDebug = n.advertise<visualization_msgs::MarkerArray>("debug/features", 1);
    const FeatureConfig config = {
            .lines={.maxError=.5, .minPoints=3},
            .arcs={.minMean=math::deg2rad(30), .maxMean=math::deg2rad(135), .maxStdev=0.15},
            .segments={.maxDist=0.10, .minSegmentAperture=30}
    };
    FeatureExtractionNode node(markerDebug, config);
    const auto &sub = n.subscribe(LIDAR_TOPIC, 10, &FeatureExtractionNode::scanReceived, &node);

    ros::spin();
}

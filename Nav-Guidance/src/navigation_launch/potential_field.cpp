#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace grid_map;
using namespace ros;

const float distSq(int x, int y, float dim) {
    auto i = (x - dim);
    auto j = (y - dim);
    return i * i + j * j;
}

const cv::Mat createLinearKernel(int dim) {
    auto mat = cv::Mat(dim, dim, CV_32F);

    for (int i = 0; i < dim; ++i) {
        for (int j = 0; j < dim; ++j) {
            auto &p = mat.at<float>(i, j);
            p = 1 - distSq(i, j, dim / 2.0F) / (dim * dim / 4.0F);
        }
    }

    return mat;
}

const cv::Mat createAttractiveKernel() {
    const auto kernelSize = 99; // MAP_SIZE_PIXELS - 1
    return createLinearKernel(kernelSize);
}

const cv::Mat createRepulsiveKernel() {
    const auto kernelSize = 13;
    return cv::Mat::ones(kernelSize, kernelSize, CV_32F) / (float) (kernelSize * kernelSize);
}

const void calculatePotential(const cv::Mat &src, cv::Mat &dst, const cv::Mat &kernel) {
    const auto anchor = cv::Point(-1, -1);
    const auto delta = 0;
    const auto ddepth = -1;
    cv::filter2D(src, dst, ddepth, kernel, anchor, delta, cv::BORDER_DEFAULT);
}

class PotentialFieldNode {

private:
    const Publisher &gridPub;
    const Publisher &pathPub;
    const cv::Mat repulsiveKernel;
    const cv::Mat attractiveKernel;

public:
    PotentialFieldNode(const Publisher &gridPub, const Publisher &pathPub)
            : gridPub(gridPub),
              pathPub(pathPub),
              repulsiveKernel(createRepulsiveKernel()),
              attractiveKernel(createAttractiveKernel()) {}

    void mapUpdated(const nav_msgs::OccupancyGrid &msg) {
        const auto repulsiveLayer = "repulsivePotential";
        const auto attractiveLayer = "attractivePotential";
        const std::string potential = "potential";

        // create map from OccupancyGrid
        GridMap map({repulsiveLayer, attractiveLayer, potential});
        GridMapRosConverter::fromOccupancyGrid(msg, repulsiveLayer, map);

        // convert to OpenCV image
        cv::Mat mapImg;
        GridMapCvConverter::toImage<unsigned short, 1>(map, repulsiveLayer, CV_16UC1, 0.0, 0.3, mapImg);

        // calculate repulsive potential
        cv::Mat repulsivePotential;
        int size = 10;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                cv::Size(2 * size + 1, 2 * size + 1),
                                                    cv::Point(size, size));
        cv::Mat closed;
        cv::morphologyEx(mapImg, closed, cv::MORPH_CLOSE, element);
        calculatePotential(closed, repulsivePotential, repulsiveKernel);
        GridMapCvConverter::addLayerFromImage<unsigned short, 1>(repulsivePotential, repulsiveLayer, map, 0.0, 1.0);

        // create artificial attractor north of origin
        cv::Mat attractors = cv::Mat::zeros(mapImg.rows, mapImg.cols, CV_16UC1);
        attractors.at<unsigned short>(0, mapImg.cols / 2) = USHRT_MAX;

        // calculate attractive potential
        cv::Mat attractivePotential;
        calculatePotential(attractors, attractivePotential, attractiveKernel);
        GridMapCvConverter::addLayerFromImage<unsigned short, 1>(attractivePotential, attractiveLayer, map, 0.0, 0.3);

        // compute aggregate potential
        map[potential] = map[repulsiveLayer] - map[attractiveLayer];

        ROS_INFO("Created grid_map with size %f x %f m (%i x %i cells).",
                 map.getLength().x(), map.getLength().y(),
                 map.getSize()(0), map.getSize()(1));

        map.setFrameId("map");
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map, message);
        gridPub.publish(message);

        cv_bridge::CvImage outImg;
        GridMapRosConverter::toCvImage(map, potential, sensor_msgs::image_encodings::MONO16, outImg);
        pathPub.publish(outImg.toImageMsg());
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_field");
    ros::NodeHandle n;

    auto pub = n.advertise<grid_map_msgs::GridMap>("/grid", 1, true);
    auto pathPub = n.advertise<cv_bridge::CvImage>("/potential_field", 1, true);
    PotentialFieldNode node{pub, pathPub};

    ros::Subscriber sub = n.subscribe("/map", 1, &PotentialFieldNode::mapUpdated, &node);
    ros::spin();

    return 0;
}


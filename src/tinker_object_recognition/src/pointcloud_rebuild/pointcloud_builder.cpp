//
// Created by 郭嘉丞 on 15/9/12.
//

#include "tinker_object_recognition/pointcloud_rebuild/pointcloud_builder.h"
#include <cstdio>

namespace tinker
{
namespace vision
{
using std::cerr;
using std::cout;
using std::endl;
using std::vector;

static const short kMinDepth = 100;

PointCloudPtr BuildPointCloud(const cv::Mat & depthImage, const cv::Mat & rgbImage) {
    PointCloudPtr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int y = 0; y < rgbImage.rows; y++) {
        for (int x = 0; x < rgbImage.cols; x++) {
            cv::Vec3s location = depthImage.at<cv::Vec3s>(y, x);
            if (location[2] < kMinDepth)
            {
                continue;
            }
            pcl::PointXYZRGB newPoint;
            newPoint.b = rgbImage.at<cv::Vec3b>(y, x)[0];
            newPoint.g = rgbImage.at<cv::Vec3b>(y, x)[1];
            newPoint.r = rgbImage.at<cv::Vec3b>(y, x)[2];
            newPoint.x = ((float)location[0]) / 1000.;
            newPoint.y = ((float) location[1]) / 1000.;
            newPoint.z = ((float) location[2]) / 1000.;
            pointCloud->points.push_back(newPoint);
        }
    }
    pointCloud->width = (int) pointCloud->points.size();
    pointCloud->height = 1;
    return pointCloud;
}

}
}

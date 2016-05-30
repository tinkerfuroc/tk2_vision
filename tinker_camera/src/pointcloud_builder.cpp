//
// Created by 郭嘉丞 on 15/9/12.
//

#include "tinker_camera/pointcloud_builder.h"
#include <cstdio>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <cfloat>

namespace tinker {
namespace vision {
using std::cerr;
using std::cout;
using std::endl;
using std::vector;

static const short kMinDepth = 100;

PointCloudPtr BuildPointCloud(const cv::Mat& depthImage,
                              const cv::Mat& rgbImage) {
    return BuildPointCloud(depthImage, rgbImage, -FLT_MAX, FLT_MAX);
}

PointCloudPtr BuildPointCloud(const cv::Mat& depthImage,
                              const cv::Mat& rgbImage, 
                              float min_x,
                              float max_x) {
    PointCloudPtr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int y = 0; y < rgbImage.rows; y++) {
        for (int x = 0; x < rgbImage.cols; x++) {
            cv::Vec3s location = depthImage.at<cv::Vec3s>(y, x);
            if (location[2] < kMinDepth) {
                continue;
            }
            pcl::PointXYZRGB newPoint;
            newPoint.b = rgbImage.at<cv::Vec3b>(y, x)[0];
            newPoint.g = rgbImage.at<cv::Vec3b>(y, x)[1];
            newPoint.r = rgbImage.at<cv::Vec3b>(y, x)[2];
            // Transform coordinate
            newPoint.x = ((float)location[2]) / 1000.;
            newPoint.y = ((float)location[0]) / 1000.;
            newPoint.z = ((float)location[1]) / 1000.;
            if (newPoint.x >= min_x && newPoint.x <= max_x)
                pointCloud->points.push_back(newPoint);
        }
    }
    pointCloud->width = (int)pointCloud->points.size();
    pointCloud->height = 1;
    return pointCloud;
}

LocPointCloudPtr BuildPointCloud(const cv::Mat& depthImage) {
    LocPointCloudPtr pointCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int y = 0; y < depthImage.rows; y++) {
        for (int x = 0; x < depthImage.cols; x++) {
            cv::Vec3s location = depthImage.at<cv::Vec3s>(y, x);
            if (location[2] < kMinDepth) {
                continue;
            }
            pcl::PointXYZ newPoint;
            // Transform coordinate
            newPoint.x = ((float)location[2]) / 1000.;
            newPoint.y = ((float)location[0]) / 1000.;
            newPoint.z = ((float)location[1]) / 1000.;
            pointCloud->points.push_back(newPoint);
        }
    }
    pointCloud->width = (int)pointCloud->points.size();
    pointCloud->height = 1;
    return pointCloud;
}

sensor_msgs::PointCloud2 ToROSCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud) {
    sensor_msgs::PointCloud2 cloud;
    pcl::PCLPointCloud2 pcl_cloud;
    pcl::toPCLPointCloud2(point_cloud, pcl_cloud);
    pcl_conversions::fromPCL(pcl_cloud, cloud);
    return cloud;
}

sensor_msgs::PointCloud2 ToROSCloud(
    const pcl::PointCloud<pcl::PointXYZ>& point_cloud) {
    sensor_msgs::PointCloud2 cloud;
    pcl::PCLPointCloud2 pcl_cloud;
    pcl::toPCLPointCloud2(point_cloud, pcl_cloud);
    pcl_conversions::fromPCL(pcl_cloud, cloud);
    return cloud;
}
}
}

//
// Created by 郭嘉丞 on 15/9/12.
// Last edited by sugar10w, 2016.4.8
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
static const short kMaxDepth = 10000;

PointCloudPtr BuildPointCloud(const cv::Mat & depthImage, const cv::Mat & rgbImage) {
    PointCloudPtr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pointCloud->width = rgbImage.cols;
    pointCloud->height = rgbImage.rows;
    pointCloud->is_dense = false;
    pointCloud->points.resize(pointCloud->width * pointCloud->height);
    
    PointT* pt = &pointCloud->points[0];

    for (int y = 0; y < rgbImage.rows; y++) {
        for (int x = 0; x < rgbImage.cols; x++) {
            cv::Vec3s location = depthImage.at<cv::Vec3s>(y, x);
            if (location[2] < kMinDepth || location[2] > kMaxDepth)
            {                
                location[0] = location[1] = location[2] = 0;
            }
            pcl::PointXYZRGB newPoint;
            newPoint.b = rgbImage.at<cv::Vec3b>(y, x)[0];
            newPoint.g = rgbImage.at<cv::Vec3b>(y, x)[1];
            newPoint.r = rgbImage.at<cv::Vec3b>(y, x)[2];
            newPoint.x = ((float)location[0]) / 1000.;
            newPoint.y = ((float) location[1]) / 1000.;
            newPoint.z = ((float) location[2]) / 1000.;
            
            *pt = newPoint;
            ++pt;
        }
    }
    return pointCloud;
}

}
}

//
// Created by 郭嘉丞 on 15/9/12.
//

#ifndef __TINKER_CAMER_POINTCLOUDBUILDER_H__
#define __TINKER_CAMER_POINTCLOUDBUILDER_H__

#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace tinker
{
namespace vision
{

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr LocPointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
PointCloudPtr BuildPointCloud(const cv::Mat & depthImage, const cv::Mat & rgbImage);
PointCloudPtr BuildPointCloud(const cv::Mat & depthImage, const cv::Mat & rgbImage, float min_x, float max_x);
LocPointCloudPtr BuildPointCloud(const cv::Mat & depthImage);
LocPointCloudPtr BuildPointCloud(const cv::Mat & depthImage);
sensor_msgs::PointCloud2 ToROSCloud(const pcl::PointCloud<pcl::PointXYZRGB> & point_cloud);
sensor_msgs::PointCloud2 ToROSCloud(const pcl::PointCloud<pcl::PointXYZ> & point_cloud);


}
}


#endif //KINECTDATAANALYZER_POINTCLOUDBUILDER_H

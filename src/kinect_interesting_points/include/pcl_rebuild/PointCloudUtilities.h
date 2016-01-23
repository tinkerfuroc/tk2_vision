//
// Created by 郭嘉丞 on 15/9/21.
//

#ifndef POINTCLOUDUTILITIES_H
#define POINTCLOUDUTILITIES_H

#include "pcl_rebuild/common.h"
#include <opencv2/opencv.hpp>

namespace tinker
{
namespace vision
{
    cv::Mat ReloadDepthImage(const char * filename);

    PointCloudPtr GetSubXCloud(PointCloudPtr cloud, double fromX, double toX);

    PointCloudPtr GetSubYCloud(PointCloudPtr cloud, double fromY, double toY);

    PointCloudPtr GetSubZCloud(PointCloudPtr cloud, double fromZ, double toZ);

    PointCloudPtr GetSubCloud(PointCloudPtr cloud, double fromX, double toX,
                              double fromY, double toY, double fromZ, double toZ);

    void RGBVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
}
}

#endif //OBJECTFINDER_POINTCLOUDUTILITIES_H

//
// Created by 郭嘉丞 on 15/9/25.
//

#ifndef OBJECTFINDER_IMAGEREBUILD_H
#define OBJECTFINDER_IMAGEREBUILD_H

#include <opencv2/opencv.hpp>
#include "pcl_rebuild/common.h"

namespace tinker
{
namespace vision
{   
extern const int kLowResWidth;
extern const int kLowResHeight, kLowResTop, kLowResBottom;
extern const int kHighResWidth, kHighResLeft, kHighResRight;
extern const int kHighResHeight;
extern const int kLowResActualHeight, kHighResActualWidth;

cv::Mat Get2DImageFromPointCloud(PointCloudPtr cloud);

cv::Mat GetHDImageFromPointCloud(PointCloudPtr cloud, cv::Mat & totalImage, bool hiRes = false);
}
}

#endif //OBJECTFINDER_IMAGEREBUILD_H

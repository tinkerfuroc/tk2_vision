//
// Created by 郭嘉丞 on 15/9/12.
//

#ifndef KINECTDATAANALYZER_POINTCLOUDBUILDER_H
#define KINECTDATAANALYZER_POINTCLOUDBUILDER_H

#include <opencv2/opencv.hpp>
#include "tinker_object_recognition/common.h"
#include <vector>

namespace tinker
{
namespace vision
{
#define DEPTH_IMAGE_ROWS 424
#define DEPTH_IMAGE_COLS 512
#define MIN_DEPTH_CM 60


PointCloudPtr BuildPointCloud(const cv::Mat & depthImage, const cv::Mat & rgbImage);


}
}


#endif //KINECTDATAANALYZER_POINTCLOUDBUILDER_H

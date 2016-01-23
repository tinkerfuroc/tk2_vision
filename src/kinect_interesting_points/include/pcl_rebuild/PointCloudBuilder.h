//
// Created by 郭嘉丞 on 15/9/12.
//

#ifndef KINECTDATAANALYZER_POINTCLOUDBUILDER_H
#define KINECTDATAANALYZER_POINTCLOUDBUILDER_H

#include <opencv2/opencv.hpp>
#include "pcl_rebuild/common.h"

namespace tinker
{
namespace vision
{
#define DEPTH_IMAGE_ROWS 424
#define DEPTH_IMAGE_COLS 512
#define MIN_DEPTH_CM 60

class PointCloudBuilder
{
public:
    PointCloudBuilder(const cv::Mat & depthMatrix, const cv::Mat & imageMatrix);

    PointCloudPtr GetPointCloud();

    cv::Mat getPointXY(int pixelX, int pixelY, double depth, double p[3][4]);
protected:
    virtual void BuildPointCloud();

    cv::Mat depthMat;
    cv::Mat imageMat;
    cv::Mat originalImage;
    PointCloudPtr pointCloud;
};
}
}


#endif //KINECTDATAANALYZER_POINTCLOUDBUILDER_H

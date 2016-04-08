#ifndef __EC_CLUSTER_DIVIDER_H__
#define __EC_CLUSTER_DIVIDER_H__

#include "opencv2/opencv.hpp"
#include "tinker_object_recognition/common.h"

namespace tinker
{
namespace vision
{

class ClusterDividerEC
{
public:
    ClusterDividerEC(PointCloudPtr point_cloud);
    virtual std::vector<PointCloudPtr> GetDividedPointClouds();
private:
    PointCloudPtr point_cloud_;
};

}
}

#endif


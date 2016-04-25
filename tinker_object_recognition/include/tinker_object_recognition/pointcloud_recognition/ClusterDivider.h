#ifndef __CLUSTER_DIVIDER_H__
#define __CLUSTER_DIVIDER_H__

#include "opencv2/opencv.hpp"
#include "tinker_object_recognition/common.h"

namespace tinker
{
namespace vision
{

class ClusterDivider
{
public:
    ClusterDivider(PointCloudPtr point_cloud);
    virtual std::vector<PointCloudPtr> GetDividedPointClouds();
private:
    PointCloudPtr point_cloud_;
};

}
}

#endif


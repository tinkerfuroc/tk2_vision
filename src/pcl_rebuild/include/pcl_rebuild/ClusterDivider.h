#ifndef __CLUSTER_DIVIDER_H__
#define __CLUSTER_DIVIDER_H__

#include "pcl_rebuild/IPointCloudDivider.h"

namespace tinker
{
namespace vision
{
class ClusterDivider:public IPointCloudDivider
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


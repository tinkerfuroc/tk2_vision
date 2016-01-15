//
// Created by 郭嘉丞 on 15/9/21.
//

#ifndef IPOINTCLOUDDIVIDER_H
#define IPOINTCLOUDDIVIDER_H

#include <vector>
#include "pcl_rebuild/common.h"

namespace tinker
{
namespace vision
{
class IPointCloudDivider
{
public:
    virtual std::vector<PointCloudPtr> GetDividedPointClouds() = 0;
    virtual ~IPointCloudDivider() { }
};
}
}


#endif //OBJECTFINDER_IPOINTCLOUDDIVIDER_H

/*
 * Created by sugar10w, 2016.4.8
 *
 * 从结构化点云，分离物体
 *
 */

#ifndef __CLUSTER_DIVIDER_H_
#define __CLUSTER_DIVIDER_H_

#include "opencv2/opencv.hpp"
#include "tinker_object_recognition/common.h"

namespace tinker {
namespace vision {

class ClusterDivider
{
public:
    ClusterDivider(PointCloudPtr point_cloud);
    std::vector<PointCloudPtr> GetDividedPointClouds();
private:
    PointCloudPtr point_cloud_;    
};

}
}

#endif //__CLUSTER_DIVIDER_H_

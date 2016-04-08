/*
 * Created by sugar10w, 2016.3.24
 * Last edited by sugar10w, 2016.3.24
 *
 * 用二维的方式将一个点云拆分为多个点云
 */

#ifndef __2D_CLUSTER_DIVIDER_H__
#define __2D_CLUSTER_DIVIDER_H__

#include <opencv2/opencv.hpp>

#include "tinker_object_recognition/common.h"
#include "tinker_object_recognition/pointcloud_recognition/object_cluster.h"

namespace tinker {
namespace vision {

class ClusterDivider2D
{
public:
    ClusterDivider2D(PointCloudPtr & cloud, cv::Mat mask);
    std::vector<ObjectCluster> GetDividedCluster();
    std::vector<PointCloudPtr> GetDividedPointClouds(); //TODO ???
    inline cv::Mat GetMask() {return mask_;}
private:
    PointCloudPtr cloud_;
    cv::Mat mask_;
};

}   
}    

#endif // __2D_CLUSTER_DIVIDER_H__

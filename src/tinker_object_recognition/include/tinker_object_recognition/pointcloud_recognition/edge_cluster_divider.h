/* 
 * Created by sugar10w, 2016.3.24
 * Last edited by sugar10w, 2016.4.8
 *
 * 查找物体：
 *  直接用shard(RGBD中的不连续点)图层获取画面轮廓，
 *  之后参考no_plane图层查看这些轮廓是否为大平面，
 *  将所有非大平面的轮廓作为物体返回；
 * 
 *  另外，找到东西之后，会在shard_mask上做标记，
 *  shard_mask相对应的区域会被删掉
 *
 *  这个方法适用于书架、水平拍摄的书桌：
 *  在画面中，水平面总是在shard_mask中就显示出轮廓来，
 *  而竖直平面在no_plane中显示为平面
 */

#ifndef __EDGE_CLUSTER_DIVIDER_H__
#define __EDGE_CLUSTER_DIVIDER_H__

#include <opencv2/opencv.hpp>
#include "tinker_object_recognition/pointcloud_recognition/object_cluster.h"
#include "tinker_object_recognition/common.h"

namespace tinker {
namespace vision {

class EdgeClusterDivider
{
public:
    EdgeClusterDivider(PointCloudPtr & cloud, cv::Mat & shard_mask, cv::Mat & no_plane_mask);
    std::vector<ObjectCluster> GetDividedCluster();
    std::vector<PointCloudPtr> GetDividedPointClouds(); //TODO ???
    inline cv::Mat GetMask() { return mask_; }
    inline cv::Mat GetShardMask() { return shard_mask_; }
private:
    PointCloudPtr cloud_;
    cv::Mat shard_mask_;
    cv::Mat no_plane_mask_;
    cv::Mat mask_;

};

}
}    

#endif // __EDGE_CLUSTER_DIVIDER_H__


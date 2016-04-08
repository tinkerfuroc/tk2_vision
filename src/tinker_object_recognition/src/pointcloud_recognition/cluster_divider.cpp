/*
 * Created by sugar10w, 2016.4.8
 *
 * 从结构化点云，分离物体
 *
 */

#include "tinker_object_recognition/pointcloud_recognition/cluster_divider.h"

#include "tinker_object_recognition/pointcloud_recognition/2d_cluster_divider.h"
#include "tinker_object_recognition/pointcloud_recognition/edge_cluster_divider.h"
#include "tinker_object_recognition/pointcloud_recognition/object_cluster.h"
#include "tinker_object_recognition/pointcloud_filter/rgbd_plane_detector.h"
#include "tinker_object_recognition/pointcloud_filter/2dmask_filter.h"

namespace tinker
{
namespace vision
{
    using std::vector;

    ClusterDivider::ClusterDivider(PointCloudPtr point_cloud)
        :point_cloud_(point_cloud)
    { }

    std::vector<PointCloudPtr> ClusterDivider::GetDividedPointClouds()
    {
        /* 去平面 */
        cv::Mat no_plane_mask = GetNoPlaneMask(point_cloud_);
        OpenImage(no_plane_mask, 0, 2, 0);

        /* 不连续点 */
        cv::Mat shard_mask = ~GetShardMask(point_cloud_);
        //OpenImage(shard_mask, 0, 1, 0);

        /* EdgeClusterDivider */
        EdgeClusterDivider pre_cluster_divider(point_cloud_, shard_mask, no_plane_mask);
        std::vector<ObjectCluster> pre_clusters = pre_cluster_divider.GetDividedCluster();
        shard_mask = pre_cluster_divider.GetShardMask();

        /* ClusterDivider2D */
        OpenImage(no_plane_mask, 0, 2, 0);
        cv::Mat mask = shard_mask & no_plane_mask;
        ClusterDivider2D cluster_divider(point_cloud_, mask);
        std::vector<ObjectCluster> clusters;
        
        /* 判定是否进行第2步 */
        if (pre_clusters.size() <= 5)
        {
            clusters = cluster_divider.GetDividedCluster();
            /* 综合两种提取方式的结果 */
            mask = pre_cluster_divider.GetMask() | cluster_divider.GetMask();
        }
        else 
        {
            mask = pre_cluster_divider.GetMask();
        }
        
        //点云过滤得到的结果
        //PointCloudPtr display_cloud = GetCloudFromMask(mask, cloud);

        std::vector<PointCloudPtr> clouds;
        for (int i=0; i<pre_clusters.size(); ++i) clouds.push_back(pre_clusters[i].GetCloud());
        for (int i=0; i<clusters.size(); ++i) clouds.push_back(clusters[i].GetCloud());
        return clouds;
    } 


}
}


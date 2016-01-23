#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl_rebuild/ImageRebuild.h"
#include "pcl_rebuild/ClusterDivider.h"

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
        PointCloudPtr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud (point_cloud_);
        vg.setLeafSize (1.f, 1.f, 1.f);
        vg.filter (*cloud_filtered);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (4); // 4cm
        ec.setMinClusterSize (150);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        vector<PointCloudPtr> divided_clouds;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            divided_clouds.push_back(cloud_cluster);
        }
        return divided_clouds;
    } 


}
}


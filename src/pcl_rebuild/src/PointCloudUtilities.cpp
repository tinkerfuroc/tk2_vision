//
// Created by 郭嘉丞 on 15/9/21.
//

#include "pcl_rebuild/PointCloudUtilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <cstdio>

namespace tinker
{
namespace vision
{
    cv::Mat ReloadDepthImage(const char * filename)
    {
        fstream fin(filename);
        int num_rows, num_cols;
        fin.read((char *) &num_rows, sizeof(int));
        fin.read((char *) &num_cols, sizeof(int));
        cv::Mat mat = cv::Mat::zeros(num_rows, num_cols, CV_32FC1);
        fin.read((char *) mat.data, num_cols * num_rows * 4);
        return mat;
    }

    void RGBVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
    {
        // --------------------------------------------
        // -----Open 3D viewer and add point cloud-----
        // --------------------------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    PointCloudPtr GetSubXCloud(PointCloudPtr cloud, double fromX, double toX)
    {
        PointCloudPtr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (unsigned i=0; i<cloud->points.size(); i++)
        {
            const pcl::PointXYZRGB & point = cloud->points[i];
            if (point.x < toX && point.x > fromX)
            {
                newCloud->points.push_back(point);
            }
        }
        newCloud->width = (int) newCloud->points.size();
        newCloud->height = 1;
        return newCloud;
    }

    PointCloudPtr GetSubYCloud(PointCloudPtr cloud, double fromY, double toY)
    {
        PointCloudPtr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (unsigned i=0; i<cloud->points.size(); i++)
        {
            const pcl::PointXYZRGB & point = cloud->points[i];
            if (point.y < toY && point.y > fromY)
            {
                newCloud->points.push_back(point);
            }
        }
        newCloud->width = (int) newCloud->points.size();
        newCloud->height = 1;
        return newCloud;
    }

    PointCloudPtr GetSubZCloud(PointCloudPtr cloud, double fromZ, double toZ)
    {
        PointCloudPtr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (unsigned i=0; i<cloud->points.size(); i++)
        {
            const pcl::PointXYZRGB & point = cloud->points[i];
            if (point.z < toZ && point.z > fromZ)
            {
                newCloud->points.push_back(point);
            }
        }
        newCloud->width = (int) newCloud->points.size();
        newCloud->height = 1;
        return newCloud;
    }

    PointCloudPtr GetSubCloud(PointCloudPtr cloud, double fromX, double toX,
                              double fromY, double toY, double fromZ, double toZ)
    {
        PointCloudPtr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (unsigned i=0; i<cloud->points.size(); i++)
        {
            const pcl::PointXYZRGB & point = cloud->points[i];
            if (point.x < toX && point.x > fromX
                && point.y < toY && point.y > fromY
                && point.z < toZ && point.z > fromZ)
            {
                newCloud->points.push_back(point);
            }
        }
        newCloud->width = (int) newCloud->points.size();
        newCloud->height = 1;
        return newCloud;
    }
}
}

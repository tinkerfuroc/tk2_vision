/*
 * Created by sugar10w, 2016.3.24
 * Last edited by sugar10w, 2016.3.24
 *
 * 用二维的方式将一个点云拆分为多个点云
 */


#include "tinker_object_recognition/pointcloud_recognition/2d_cluster_divider.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tinker_object_recognition/pointcloud_filter/2dmask_filter.h"

namespace tinker {
namespace vision {


ClusterDivider2D::ClusterDivider2D(PointCloudPtr & cloud, cv::Mat mask)
    : cloud_(cloud), mask_(mask)
{
}

std::vector<ObjectCluster> ClusterDivider2D::GetDividedCluster()
{
    std::vector<ObjectCluster> divided_clouds;

    cv::Mat contour = ~mask_;

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( contour, contours, CV_RETR_CCOMP , CV_CHAIN_APPROX_NONE);

    for (int idx=0; idx<contours.size(); ++idx)
    {
        double cont_size = cv::contourArea(contours[idx]);
        if (cont_size>20000) continue;
        cv::Mat inside = cv::Mat::zeros(mask_.size(), CV_8UC3);
        cv::Mat sub_mask;
        cv::drawContours(inside, contours, idx, cv::Scalar(255,255,255), CV_FILLED, 4);
        cv::cvtColor(inside, sub_mask, CV_RGB2GRAY);

        if (cont_size<300)
        {
            mask_ &= ~sub_mask;
            continue;
        }

        ObjectCluster object_cluster( GetCloudFromMask(sub_mask, cloud_));
        if (object_cluster.isValid())
        {
            divided_clouds.push_back(object_cluster);
        }
        else
        {
            mask_ &= ~sub_mask;
        }

    }

    return divided_clouds;
}

std::vector<PointCloudPtr> ClusterDivider2D::GetDividedPointClouds()
{
    std::vector<ObjectCluster> clusters = GetDividedCluster();
    std::vector<PointCloudPtr> clouds;
    for (int i=0; i<clusters.size(); ++i)
        clouds.push_back(clusters[i].GetCloud());
    return clouds;
}

}    
}    

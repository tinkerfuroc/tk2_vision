#include "tinker_object_recognition/utilities.h"
//#include <boost/foreach.hpp>
#include <vector>

using std::vector;

namespace tinker {
namespace vision {

geometry_msgs::Point GetCenter(PointCloudPtr point_cloud) {
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    BOOST_FOREACH (pcl::PointXYZRGB point, point_cloud->points) {
        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;
    }
    double num_points = point_cloud->points.size();
    geometry_msgs::Point center;
    center.x = sum_x / num_points;
    center.y = sum_y / num_points;
    center.z = sum_z / num_points;
    return center;
}

sensor_msgs::PointCloud2 ToROSCloud(PointCloudPtr point_cloud) {
    sensor_msgs::PointCloud2 cloud;
    pcl::PCLPointCloud2 pcl_cloud;
    pcl::toPCLPointCloud2(*point_cloud, pcl_cloud);
    pcl_conversions::fromPCL(pcl_cloud, cloud);
    return cloud;
}

cv::Mat HistogramEqualizeRGB(const cv::Mat& src) {
    cv::Mat ycrcb;
    cv::cvtColor(src, ycrcb, CV_BGR2YCrCb);
    vector<cv::Mat> channels;
    cv::split(ycrcb, channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::Mat dst;
    cv::merge(channels, ycrcb);
    cvtColor(ycrcb, dst, CV_YCrCb2BGR);
    return dst;
}

}
}

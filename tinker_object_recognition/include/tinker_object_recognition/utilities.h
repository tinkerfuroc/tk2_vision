#ifndef __TINKER_OBJECT_RECOGNITION_UTILITIES__
#define __TINKER_OBJECT_RECOGNITION_UTILITIES__

#include "tinker_object_recognition/common.h"
#include <geometry_msgs/Point.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/core.hpp>

namespace tinker {
namespace vision {

geometry_msgs::Point GetCenter(PointCloudPtr point_cloud);
cv::Mat HistogramEqualizeRGB(const cv::Mat & src);

}
}

#endif

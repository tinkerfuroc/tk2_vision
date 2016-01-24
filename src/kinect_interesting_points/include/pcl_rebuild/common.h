//
// Created by 郭嘉丞 on 15/9/12.
//

#ifndef KINECTDATAANALYZER_COMMON_H
#define KINECTDATAANALYZER_COMMON_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace tinker
{
namespace vision
{
#define __DEBUG__

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
}
}


#endif //KINECTDATAANALYZER_COMMON_H

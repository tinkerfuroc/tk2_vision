#ifndef __TINKER_VISION_COMMON_H__
#define __TINKER_VISION_COMMON_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <XmlRpc.h>

namespace tinker {
namespace vision {
//#define __DEBUG__

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef XmlRpc::XmlRpcValue::ValueStruct::iterator XmlIter;
typedef XmlRpc::XmlRpcValue::ValueStruct::const_iterator XmlcIter;
}
}

#endif  // KINECTDATAANALYZER_COMMON_H

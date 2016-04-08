#ifndef __TINKER_VISION_COMMON_H__
#define __TINKER_VISION_COMMON_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <XmlRpc.h>

namespace tinker {
namespace vision {
//#define __DEBUG__

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef PointCloudNT::Ptr PointCloudNTPtr;
typedef PointCloudNT::ConstPtr PointCloudNTConstPtr;

typedef XmlRpc::XmlRpcValue::ValueStruct::iterator XmlIter;
typedef XmlRpc::XmlRpcValue::ValueStruct::const_iterator XmlcIter;
}
}

#endif  // __TINKER_VISION_COMMON_H__

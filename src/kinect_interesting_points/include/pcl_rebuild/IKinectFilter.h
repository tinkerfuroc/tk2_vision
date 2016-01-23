#ifndef __IKINECT_FILTER_H__
#define __IKINECT_FILTER_H__

#include "pcl_rebuild/common.h"
#include <opencv2/opencv.hpp>

namespace tinker
{
namespace vision
{
    class IKinectFilter
    {
    public:
        void Filter(cv::Mat & depth_mat, cv::Mat & image_mat);
        virtual ~IKinectFilter() { }
    };
}
}

#endif


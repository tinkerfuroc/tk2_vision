#ifndef __LINE_FILTER_H__
#define __LINE_FILTER_H__

#include "pcl_rebuild/IKinectFilter.h"
#include "pcl_rebuild/common.h"

namespace tinker
{
namespace vision
{
    class LineFilter:public IKinectFilter
    {
    public:
        void Filter(cv::Mat & depth_mat, cv::Mat & image_mat);
    private:
        void RemoveLines(cv::Mat & depth_mat, cv::Mat & image_mat);
    };
}
}

#endif


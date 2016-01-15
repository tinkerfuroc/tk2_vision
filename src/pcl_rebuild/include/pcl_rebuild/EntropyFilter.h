#ifndef __ENTROPY_FILTER_H__
#define __ENTROPY_FILTER_H__

#include "pcl_rebuild/IKinectFilter.h"
#include "pcl_rebuild/common.h"

namespace tinker
{
namespace vision
{

class EntropyFilter:public IKinectFilter
{
public:
    EntropyFilter(int filter_size, double threshold);
    void Filter(cv::Mat & depth_mat, cv::Mat & image_mat);
    ~EntropyFilter();
private:
    cv::Mat GetEntropyImage(const cv::Mat & image_mat);
    void FilterEntropy(double threshold, cv::Mat & entropy_mat, cv::Mat & depth_mat, cv::Mat & image_mat);
    void OpenDepthImage(cv::Mat & depth_mat);
    void BuildEntropyTable();

    double * entropy_table_;
    int filter_size_;
    double threshold_;
};

}
}

#endif


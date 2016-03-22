#ifndef __TINKER_OBJECT_RECOGNITION_FILTERS_H__
#define __TINKER_OBJECT_RECOGNITION_FILTERS_H__

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

namespace tinker {
namespace vision{

//All mask image returned are CV_8UC1, all input should be grayscale
//Pixels with mask value 0 will be removed when applying the mask
cv::Mat EntropyFilterMask(const cv::Mat & image, float threshold, int filter_size, int jump_step);
cv::Mat EntropyFilterMask(const cv::Mat & image, float threshold, int filter_size, const std::vector<float> & entropy_table, int jump_step);
std::vector<float> BuildEntropyTable(int filter_size);
void DilateImage(cv::Mat & image, int kernel_size);
void ErodeImage(cv::Mat & image, int kernel_size);
cv::Mat LineFilterMask(const cv::Mat & image, double linewidth);

template<typename T>
void ApplyMask(const cv::Mat & mask, cv::Mat & image, const T & none_value) {
    for(int i = 0; i < image.rows; i++) {
        for(int j = 0; j < image.cols; j++) {
            if (!mask.at<unsigned char>(i, j)) {
                image.at<T>(i, j) = none_value;
            }
        }
    }
} 

}
}


#endif


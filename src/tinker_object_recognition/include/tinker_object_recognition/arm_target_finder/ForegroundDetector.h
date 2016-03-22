// Foreground Detector.
// Du 2016.1.15.
//
#ifndef __FOREGROUND_DETECTOR_H__
#define __FOREGROUND_DETECTOR_H__

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/core.hpp>
#include <tinker_object_recognition/graph_filter/filters.h>
#include <vector>

namespace tinker
{
    namespace vision
    {
        
        class ForegroundDetector
        {
        public:
            ForegroundDetector(int filter_size, double entropy_threshold)
                : filter_size_(filter_size), entropy_threshold_(entropy_threshold) {}
            void Filter(const cv::Mat & source_mat, cv::Mat & desk_mat);
            cv::Mat TakePhoto(const cv::Mat & source_mat);
        private:
            //processes.
            void FilterByEntropy(const cv::Mat & source_mat, cv::Mat & desk_mat);
            void OpenImage(cv::Mat & mat);
            cv::Mat BuildMask(const cv::Mat & mat);
            void DivideMaskByY(cv::Mat & mask_mat);
            void BuildImageByMask(const cv::Mat & source_mat, cv::Mat & desk_mat, cv::Mat mask_mat);
            //parameters.
            int filter_size_;
            double entropy_threshold_;
        };
        
        
    }
}

#endif

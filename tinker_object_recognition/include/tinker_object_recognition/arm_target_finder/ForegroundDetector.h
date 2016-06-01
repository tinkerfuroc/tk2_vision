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

namespace tinker {
namespace vision {

struct ForegroundImage {
    cv::Mat foreground;
    cv::Rect bound;
};

class ForegroundDetector {
public:
    ForegroundDetector() {}

    // Filter the picture and paint the background with black color
    // all black if nothing detected
    void Filter(const cv::Mat &source_mat, cv::Mat &desk_mat);

    // Filter the picture and cut the foreground out
    // keep desk_mat constant if nothing detected and return -1;
    int CutForegroundOut(const cv::Mat &source_mat, cv::Mat &desk_mat,
                         cv::Rect &bound);
    int CutForegroundOut(const cv::Mat &source_mat, cv::Mat &desk_mat);

    std::vector<ForegroundImage> CutAllForegroundOut(const cv::Mat &source_mat);
                         
    const static int DETECTED = 0;
    const static int NOT_DETECTED = -1;
    
    void setParam(ros::NodeHandle private_nh_) {
        XmlRpc::XmlRpcValue foreground_detector_param;
        private_nh_.getParam("foreground_detector_param", foreground_detector_param);
        ROS_ASSERT(foreground_detector_param.size() > 0);
        ROS_ASSERT(foreground_detector_param.hasMember("filter_size"));
        ROS_ASSERT(foreground_detector_param.hasMember("entropy_threshold"));
        ROS_ASSERT(foreground_detector_param.hasMember("max_aspect_ratio_tolerance"));
        ROS_ASSERT(foreground_detector_param.hasMember("min_squaresize"));
        ROS_ASSERT(foreground_detector_param["filter_size"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(foreground_detector_param["entropy_threshold"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(foreground_detector_param["max_aspect_ratio_tolerance"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(foreground_detector_param["min_squaresize"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        filter_size_ = (int)foreground_detector_param["filter_size"];
        entropy_threshold_ = (double)foreground_detector_param["entropy_threshold"];
        max_aspect_ratio_tolerance_ = (double)foreground_detector_param["max_aspect_ratio_tolerance"];
        min_squaresize_ = (double)foreground_detector_param["min_squaresize"];
    }

private:
    // processes.
    void FilterByEntropy(const cv::Mat &source_mat, cv::Mat &desk_mat);
    void OpenImage(cv::Mat &mat);
    cv::Mat BuildMask(const cv::Mat &mat);
    void DivideMaskByY(cv::Mat &mask_mat);
    void BuildImageByMask(const cv::Mat &source_mat, cv::Mat &desk_mat,
                          cv::Mat mask_mat);

    // parameters.
    int filter_size_;
    double entropy_threshold_;
    double max_aspect_ratio_tolerance_;
    double min_squaresize_;
};
}
}

#endif

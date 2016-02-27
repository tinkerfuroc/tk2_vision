//Object Info with zhuangbility
//Du 2015.1.18
//

#ifndef __OBJECTINFO_H__
#define __OBJECTINFO_H__

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <vector>
#include <string>

namespace tinker
{
    namespace vision
    {
        class ObjectInfo
        {
        public:
            ObjectInfo(cv::Mat img_mat)
            {
                static const int res_size = 12;
                cv::GaussianBlur(img_mat, img_mat_, cv::Size(15, 15), 10.0);
                cv::resize(img_mat_, img_mat_, cv::Size(res_size, res_size));
            }
            cv::Mat GetImage() {return img_mat_;}
        private:
            cv::Mat img_mat_;
        };
        
        
        
        class ZhuangBiObjectInfo : public ObjectInfo
        {
        public:
            ZhuangBiObjectInfo(cv::Mat img_mat, std::string name, 
                cv::Scalar color) : 
                ObjectInfo(img_mat), name_(name), color_(color) {}
            const char *GetName() {return name_.c_str();}
            const cv::Scalar GetColor() {return color_;}
        private:
            std::string name_;
            cv::Scalar color_;
        };
    }
}

#endif


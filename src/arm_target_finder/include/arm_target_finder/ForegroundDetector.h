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
#include <arm_target_finder/ObjectInfo.h>
#include <vector>

namespace tinker
{
    namespace vision
    {
        namespace dirty
        {
            struct similarity_
            {
                int object_no;
                double dotproduct;
                cv::Rect pos;
            };
        }
        using dirty::similarity_;
        
        class ForegroundDetector
        {
        public:
            ForegroundDetector(int filter_size, double entropy_threshold, double similarity_threshold);
            ~ForegroundDetector();
            void Filter(const cv::Mat & source_mat, cv::Mat & desk_mat);
            void FindObject(const cv::Mat & source_mat, bool &object_recognized, float &object_similarity, 
                const char *&object_name, cv::Rect &object_pos, ZhuangBiObjectInfo **objs, int objnum);
            cv::Mat TakePhoto(const cv::Mat & source_mat);
        private:
            //help struct
            double *entropy_table_; //the buffer of plgp for calculating entropy.
            //processes.
            similarity_ FilterByObject(const cv::Mat & source_mat, ObjectInfo **objs, int objnum);
            void FilterByEntropy(const cv::Mat & source_mat, cv::Mat & desk_mat);
            void OpenImage(cv::Mat & mat);
            cv::Mat BuildMask(const cv::Mat & mat);
            void DivideMaskByY(cv::Mat & mask_mat);
            void BuildImageByMask(const cv::Mat & source_mat, cv::Mat & desk_mat, cv::Mat mask_mat);
            //parameters.
            double similarity_threshold_;
            int filter_size_;
            double entropy_threshold_;
            //help small funcs.
            void DilateImage(cv::Mat & mat, int kernel_size);
            void ErodeImage(cv::Mat & mat, int kernel_size);
            void BuildEntropyTable_();
            inline int abs_(int n){return (n>=0)?n:(-1*n);}
            inline int sqr_(int n){return n*n;}
            inline double sqr_(double d){return d*d;}
            double dotproduct_(cv::Mat m1, cv::Mat m2);
            double dot_(cv::Mat m1, cv::Mat m2, cv::Vec3b offset = cv::Vec3b(128, 128, 128));
            cv::Vec3b bar_(cv::Mat m1, cv::Mat m2);
        };
        
        
    }
}

#endif

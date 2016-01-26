//
// Created by 郭嘉丞 on 15/10/10.
//

#include "pcl_rebuild/EntropyFilter.h"
#include <iostream>
#include <cmath>

namespace tinker
{
namespace vision
{
    using namespace std;

    EntropyFilter::EntropyFilter(int filter_size, double threshold)
        :entropy_table_(new double[filter_size*filter_size + 1]), 
        filter_size_(filter_size), threshold_(threshold)
     {
        BuildEntropyTable();
    }

    void EntropyFilter::Filter(cv::Mat & depth_mat, cv::Mat & image_mat)
     {
        cv::Mat entropy_mat = GetEntropyImage(image_mat);
        FilterEntropy(threshold_, entropy_mat, depth_mat, image_mat);
        OpenDepthImage(depth_mat);
    }

    cv::Mat EntropyFilter::GetEntropyImage(const cv::Mat & image_mat)
     {
        cv::Mat entropy_image(image_mat.rows, image_mat.cols, CV_64FC1);
        cv::Mat gray_scale_image;
        cv::cvtColor(image_mat, gray_scale_image, CV_BGR2GRAY);
        int gray_levels = filter_size_ * filter_size_;
        int *gray_scale = new int[gray_levels];
        int mov_start = filter_size_ / 2;
        for (int i = 0; i < image_mat.rows; i++)
         {
            for (int j = 0; j < image_mat.cols; j++)
             {
                if (i - mov_start < 0 || j - mov_start < 0
                    || i - mov_start + filter_size_ >= image_mat.rows ||
                    j - mov_start + filter_size_ >= image_mat.cols)
                {
                    entropy_image.at<double>(i, j) = 0;
                }
                else
                {
                    int x = i - mov_start;
                    int y = j - mov_start;
                    for (int k = 0; k < gray_levels; k++)
                    {
                        gray_scale[k] = 0;
                    }
                    for (int movx = 0; movx < filter_size_; movx++)
                        for (int movy = 0; movy < filter_size_; movy++)
                        {
                            double gray = gray_scale_image.at<uchar>(movx + x, movy + y);
                            gray = gray * ((double) gray_levels) / 256;
                            gray_scale[int(gray)]++;
                        }
                    double entropy = 0;
                    for (int k = 0; k < gray_levels; k++)
                    {
                        double add_entropy = double(gray_scale[k]) * entropy_table_[gray_scale[k]] / double(gray_levels);
                        entropy += add_entropy;
                    }
                    entropy_image.at<double>(i, j) = entropy / entropy_table_[1];
                }
            }
        }
        delete [] gray_scale;
        return entropy_image;
    }

    void EntropyFilter::FilterEntropy(double threshold, cv::Mat & entropy_mat, cv::Mat & depth_mat, cv::Mat & image_mat)
    {
        for (int i = 0; i < entropy_mat.rows; i++)
        {
            for (int j = 0; j < entropy_mat.cols; j++)
            {
                if (entropy_mat.at<double>(i, j) < threshold)
                {
                    depth_mat.at<float>(i, j) = 0;
                    image_mat.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
                }
            }
        }
        //cv::imwrite("filtered.png", image_mat);
    }

    void EntropyFilter::BuildEntropyTable()
    {
        entropy_table_[0] = 0;
        double gray_levels = filter_size_ * filter_size_;
        for (int i = 1; i <= filter_size_ * filter_size_; i++)
        {
            entropy_table_[i] = log2((double) i / gray_levels);
        }
    }

    void EntropyFilter::OpenDepthImage(cv::Mat & depth_mat)
    {
        static const int refill_kernel_size = 3;
        static const int erode_kernel_size = 5;
        static const int dilate_kernel_size = 5;
        cv::Mat refill_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(2 * refill_kernel_size + 1, 2 * refill_kernel_size + 1),
            cv::Point(refill_kernel_size, refill_kernel_size));
        cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(2 * erode_kernel_size + 1, 2 * erode_kernel_size + 1),
            cv::Point(erode_kernel_size, erode_kernel_size));
        cv::dilate(depth_mat, depth_mat, refill_kernel);
        cv::erode(depth_mat, depth_mat, erode_kernel);
        cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(2 * dilate_kernel_size + 1, 2 * dilate_kernel_size + 1),
            cv::Point(dilate_kernel_size, dilate_kernel_size));
        cv::dilate(depth_mat, depth_mat, dilate_kernel);
    }

    EntropyFilter::~EntropyFilter()
    {
        delete [] entropy_table_;
    }
}
}


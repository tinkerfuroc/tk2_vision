//
// Created by yht on 04/10/15.
//

#include "pcl_rebuild/LineFilter.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace tinker
{
namespace vision
{
    using namespace std;
    using namespace cv;

    void LineFilter::Filter(cv::Mat & depth_mat, cv::Mat & image_mat)
    {
        RemoveLines(depth_mat, image_mat);
    }
    
    cv::Mat CannyDownSample(cv::Mat &img, int downsampleStep=10)
    {
      cv::Mat d_img(img.rows/downsampleStep, img.cols/downsampleStep, CV_8UC1);
      cv::Mat o_img(img.rows/downsampleStep, img.cols/downsampleStep, CV_8UC1);   

      for (int y=0; y<d_img.rows; ++y)
        for (int x=0; x<d_img.cols; ++x) o_img.at<uchar>(y,x) = d_img.at<uchar>(y,x) = 0;

      for (int y=0; y<img.rows; ++y)
        for (int x=0; x<img.cols; ++x) 
          if (img.at<uchar>(y,x))
            d_img.at<uchar>(y/downsampleStep, x/downsampleStep)=255;
      
      for (int y=1; y<d_img.rows-1; ++y)
        for (int x=1; x<d_img.cols-1; ++x)
          if (d_img.at<uchar>(y,x)==0) 
            o_img.at<uchar>(y,x)=0;
          else
          {
            if ( d_img.at<uchar>(y-1,x) && d_img.at<uchar>(y+1,x) && d_img.at<uchar>(y,x-1) && d_img.at<uchar>(y,x+1) ) 
              o_img.at<uchar>(y,x)=0;
            else 
              o_img.at<uchar>(y,x)=255;
          }
      
      return o_img;
    }

    void LineFilter::RemoveLines(cv::Mat &depth_mat, cv::Mat & image_mat)
    {
        int downsampleStep = 4;
        Mat dst, cdst, d_dst;
        Canny(image_mat, dst, 50, 200, 3);
        d_dst = CannyDownSample(dst, downsampleStep);
        cvtColor(depth_mat, cdst, COLOR_GRAY2BGR);
#ifdef __DEBUG__
        imwrite("CannyDownsampled.png", d_dst); 
#endif

        vector<Vec4i> lines;
        HoughLinesP(d_dst, lines, 1, CV_PI / 180, 10, (int)(d_dst.rows*0.2), 3);
        for (size_t i = 0; i < lines.size(); i++)
        {
            Vec4i l = lines[i];
            l[0]*=downsampleStep; l[1]*=downsampleStep; l[2]*=downsampleStep; l[3]*=downsampleStep;
            line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 0), downsampleStep*3, 4);
#ifdef __DEBUG__
            line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), downsampleStep*3, 4);
            line(image_mat, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), downsampleStep*3, 4);
#endif
        } 
#ifdef __DEBUG__
        imwrite("CannyAfterLineFilter.png", dst);
#endif
        cvtColor(cdst, depth_mat, COLOR_RGB2GRAY);
    }
}
}

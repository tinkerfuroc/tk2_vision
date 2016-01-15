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

    void LineFilter::RemoveLines(cv::Mat &depth_mat, cv::Mat & image_mat)
    {
        Mat dst, cdst;
        Canny(image_mat, dst, 50, 200, 3);
        cvtColor(depth_mat, cdst, COLOR_GRAY2BGR);

        vector<Vec4i> lines;
        HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 50, 10);
        for (size_t i = 0; i < lines.size(); i++)
        {
            Vec4i l = lines[i];
            line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 0), 5, CV_AA);
#ifdef __DEBUG__
            line(image_mat, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 5, CV_AA);
#endif
        }
        cvtColor(cdst, depth_mat, COLOR_RGB2GRAY);
    }
}
}

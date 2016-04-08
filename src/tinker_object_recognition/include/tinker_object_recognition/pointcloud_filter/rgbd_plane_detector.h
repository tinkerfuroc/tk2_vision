/*
 * Created by sugar10w, 2016.3.19
 * Last edited by sugar10w, 2016.4.8
 *
 * 从结构化点云（二维矩阵排列的点云），
 * 获得无平面蒙版
 */

#ifndef _KINECT_DEPTH_CONTOUR__
#define _KINECT_DEPTH_CONTOUR__

#include <opencv2/opencv.hpp>
#include "tinker_object_recognition/common.h"


namespace tinker {
namespace vision {


/* 获取无平面蒙版 */
cv::Mat GetNoPlaneMask(PointCloudPtr & cloud);


}
}


#endif //_KINECT_DEPTH_CONTOUR__

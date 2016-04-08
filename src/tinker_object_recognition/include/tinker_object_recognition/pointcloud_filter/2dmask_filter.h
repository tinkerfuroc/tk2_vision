/*
 * Created by sugar10w, 2016.3.18
 * Last edited by sugar10w, 2016.4.8
 * 
 * 512*424的二维蒙版 与 512*424结构化点云 之间的操作。
 * 注：二维蒙版CV_8UC1，某个像素为0代表将被除去
 *  结构化点云 是指 width=512,height=424,且水平方向和二维蒙版的水平方向相反的 点云
 *
 * GetShardMask用于查找画面中的不连续点(相邻点距离太远的点)
 */

#ifndef _POINTCLOUD_2DMASK_FILTER__
#define _POINTCLOUD_2DMASK_FILTER__

#include "tinker_object_recognition/common.h"
#include <opencv2/opencv.hpp>

namespace tinker {
namespace vision {

/*将512*424的二维蒙版，应用到原始点云上*/
PointCloudPtr GetCloudFromMask(const cv::Mat & mask, const PointCloudConstPtr & cloud);    

/* 找到结构化点云中哪些点是有效点，返回蒙版 */
cv::Mat GetValidMask(PointCloudConstPtr cloud);

/* 对二维图片的膨胀+腐蚀+膨胀 */
void OpenImage(cv::Mat & img, 
     int refill_kernel_size,
     int erode_kernel_size,
     int dilate_kernel_size);

/* 对结构化点云的膨胀+腐蚀+膨胀 */
void OpenImage(PointCloudPtr & cloud, 
     int refill_kernel_size,
     int erode_kernel_size,
     int dilate_kernel_size);

/* 查找画面中的不连续点(相邻点距离太远的点) */
cv::Mat GetShardMask(PointCloudPtr & cloud);

}    
}

#endif //_POINTCLOUD_2DMASK_FILTER__

/*
 * Created by sugar10w, 2016.3.19
 * Last edited by sugar10w, 2016.4.8
 *
 * 从结构化点云（二维矩阵排列的点云），
 * 获得无平面蒙版
 */

#include "tinker_object_recognition/pointcloud_filter/rgbd_plane_detector.h"

#include <cmath>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/features/integral_image_normal.h> 

#include "tinker_object_recognition/pointcloud_filter/2dmask_filter.h"

namespace tinker {
namespace vision {

static float square_distance(const float* n1, const float* n2)
{
    return 
        (n1[0]-n2[0])*(n1[0]-n2[0]) +
        (n1[1]-n2[1])*(n1[1]-n2[1]) +
        (n1[2]-n2[2])*(n1[2]-n2[2]);
}
/* 根据pcl::PointXYZRGBNomal，返回边缘蒙版。 */
static cv::Mat GetEdgeFromDepth(PointCloudNTPtr & cloud)
{
    cv::Mat edge(cloud->height, cloud->width, CV_8UC1);
    
    // 注意统一这里的格式
    int k = 0;
    for (int i=0; i<cloud->height; ++i)
        for (int j=cloud->width-1; j>=0; --j)
        {
            if (j==0 || j+1==cloud->width 
                || i==0 || i+1==cloud->height)
            {
                edge.at<uchar>(i, j)=0;
                k++;
                continue;
            }

            const PointNT 
                & pt  = cloud->points[k],
                & pt1 = cloud->points[k-cloud->width],
                & pt2 = cloud->points[k+cloud->width],
                & pt3 = cloud->points[k-1],
                & pt4 = cloud->points[k+1];

            const float *nn  = pt.normal,
                        *nn1 = pt1.normal,
                        *nn2 = pt2.normal,
                        *nn3 = pt3.normal,
                        *nn4 = pt4.normal;
            
            if ( square_distance(nn, nn1) + square_distance(nn, nn2) +
                 square_distance(nn, nn3) + square_distance(nn, nn4) > 0.03f)
                edge.at<uchar>(i, j) = 255;
            else 
                edge.at<uchar>(i, j) = 0;

            k++;
        }
    return edge;
}

/* 计算轮廓的面积，清理面积小的轮廓 */
static void RoughFixMask(PointCloudPtr &cloud, cv::Mat & mask)
{
    const cv::Mat contour = ~mask;
    
    cv::Mat fill;
    cv::cvtColor(mask, fill, CV_GRAY2RGB);

    //获取轮廓
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( contour, contours, CV_RETR_CCOMP , CV_CHAIN_APPROX_NONE);

    //把面积小的直接填起来
    for (int idx=0; idx<contours.size(); ++idx)
    {
        if (contours[idx].size()>1800) continue;

        double cont_size = cv::contourArea(contours[idx]);
        if (cont_size<300) 
        {
            cv::drawContours(fill, contours, idx, cv::Scalar(255,255,255), CV_FILLED, 4);
            continue;
        }
    }

    cv::cvtColor(fill, mask, CV_RGB2GRAY);
}

/* 为了处理前端细小物体的干扰而导致平面被切分的情况，判断相邻的区域平面是否共面，再把这些区域连接起来。 */
/* 判断共线的函数 */
static pcl::PointXYZ GetVector(PointT pt1, PointT pt2, bool unit=true)
{
    pcl::PointXYZ vector;
    vector.x = pt2.x - pt1.x;
    vector.y = pt2.y - pt1.y;
    vector.z = pt2.z - pt1.z;
    if (!unit) return vector;

    float m = sqrt(vector.x*vector.x+vector.y*vector.y+vector.z*vector.z);
    vector.x /= m;
    vector.y /= m;
    vector.z /= m;
    return vector;
}
static float Distance(pcl::PointXYZ unit_vector, pcl::PointXYZ vector)
{
    pcl::PointXYZ result;
    result.x = unit_vector.y * vector.z - unit_vector.z * vector.y;
    result.y = unit_vector.z * vector.x - unit_vector.x * vector.z;
    result.z = unit_vector.x * vector.y - unit_vector.y * vector.x;
    return sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
}
static void ConnectDepthMask(PointCloudPtr & cloud, cv::Mat & mask)
{
    /* 水平方向抽取一部分线进行处理。注意此时黑色是有效点(平面点)。 */
    /* 如果左右抽取到的线段是共线的，就认为左右平面共面 */
    const int box_size = 5;
    const float dist_limit = 0.02f;

    for (int x=box_size; x<cloud->height-1; x+=box_size)
    {
        //先抽取这条水平线上所有的连续区域，[left,right]
        std::vector<cv::Vec2i> pair_list;

        for (int y=0; y<cloud->width; ++y)
        {
            if (mask.at<uchar>(x,y)) continue;
            
            //获取线段的范围
            int start = y, end = y+1;
            while ( end<cloud->width && !mask.at<uchar>(x, end)) ++end;
            --end;

            //检查线段合法性（是不是直的，是否来自平面）。无效线段就填入(0,0)。
            if (end-start<=5) { pair_list.push_back(cv::Vec2i(0,0)); y=end+1; continue;}
            bool valid = true;
            const PointT & start_point = cloud->points[(x+1)*cloud->width-start-1];
            pcl::PointXYZ unit_vector = GetVector(start_point,cloud->points[(x+1)*cloud->width-end-1]);
            for (int i=start+1; i<end; ++i)
                if (Distance(unit_vector, 
                    GetVector(start_point, cloud->points[(x+1)*cloud->width-i-1], false))
                        > dist_limit)
                {
                    valid = false;
                    break;
                }
            if (valid) pair_list.push_back(cv::Vec2i(start, end));
                 else  pair_list.push_back(cv::Vec2i(0,0));
            y = end;
        }

        //测试相邻线段是否共线并连接
        for (int i=1; i<pair_list.size(); ++i)
        {
            if (pair_list[i][0]==0 && pair_list[i][1]==0 ) continue;
            if (pair_list[i-1][0]==0 && pair_list[i-1][1]==0 ) continue;

            bool bind = true;
            PointT start_point = cloud->points[(x+1)*cloud->width-pair_list[i-1][1]-1];
            pcl::PointXYZ unit_vector = GetVector(
                cloud->points[(x+1)*cloud->width-pair_list[i-1][0]-1],
                cloud->points[(x+1)*cloud->width-pair_list[i-1][1]-1]);
            for (int j=pair_list[i][0]; j<pair_list[i][1]; ++j)
                if (Distance(unit_vector,
                    GetVector(start_point, cloud->points[(x+1)*cloud->width-j-1], false))
                        > dist_limit)
                {
                    bind = false;
                    break;
                }
            if (bind)
            {
                //一定要绘制三条线，防止误连上下区域
                for (int j=pair_list[i-1][1]; j<pair_list[i][0]; ++j) mask.at<uchar>(x-1,j)=255;
                for (int j=pair_list[i-1][1]; j<pair_list[i][0]; ++j) mask.at<uchar>(x,j)=0;
                for (int j=pair_list[i-1][1]; j<pair_list[i][0]; ++j) mask.at<uchar>(x+1,j)=255;
            }
        }
    }

}

/* 根据轮廓图像，逐个判断是否为平面并过滤 */
static cv::Mat GetMaskFromEdge(PointCloudPtr & cloud, PointCloudNTPtr & normals, cv::Mat & edge)
{
    const cv::Mat contour = ~ edge;

    cv::Mat fill;
    cv::cvtColor(edge, fill, CV_GRAY2RGB);

    //获取轮廓
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( contour, contours, CV_RETR_CCOMP , CV_CHAIN_APPROX_NONE);

    //逐个判断：三维尺寸，三维法向量
    for (int idx=0; idx<contours.size(); ++idx)
    {
        cv::Mat inside = cv::Mat::zeros(edge.size(), CV_8UC3);
        cv::drawContours(inside, contours, idx, cv::Scalar(255,255,255), CV_FILLED, 4);
        float nx_min = 2, nx_max = -2,
              ny_min = 2, ny_max = -2,
              nz_min = 2, nz_max = -2,
              x_min = 10000, x_max = -10000,
              y_min = 10000, y_max = -10000,
              z_min = 10000, z_max = -10000;
        // 注意统一这里的格式
        int k = 0;
        for (int i=0; i<cloud->height; ++i)
            for (int j=cloud->width-1; j>=0; --j)
            {
                if (inside.at<uchar>(i,j))
                {
                    const PointT  & pt  = cloud->points[k];
                    PointNT & pnt = normals->points[k];

                    if (pt.x<x_min) x_min = pt.x;
                    if (pt.x>x_max) x_max = pt.x;
                    if (pt.y<y_min) y_min = pt.y;
                    if (pt.y>y_max) y_max = pt.y;
                    if (pt.z<z_min) z_min = pt.z;
                    if (pt.z>z_max) z_max = pt.z;
                    if (pnt.normal_x<nx_min) nx_min = pnt.normal_x;
                    if (pnt.normal_x>nx_max) nx_max = pnt.normal_x;
                    if (pnt.normal_y<ny_min) ny_min = pnt.normal_y;
                    if (pnt.normal_y>ny_max) ny_max = pnt.normal_y;
                    if (pnt.normal_z<nz_min) nz_min = pnt.normal_z;
                    if (pnt.normal_z>nz_max) nz_max = pnt.normal_z;
                    
                }
                ++k;
            }
        if (! ( (x_max-x_min)>0.6
            || (y_max-y_min)>0.6
            || (z_max-z_min)>0.6
            || ( (nx_max-nx_min)<0.01 && (ny_max-ny_min)<0.01 && (nz_max-nx_min)<0.01 ))) //TODO
        {
            cv::drawContours(fill, contours, idx, cv::Scalar(255,255,255), CV_FILLED, 4);
        }
        //else
        //{
        //    OpenImage(inside, 4, 0, 0);
        //    fill &= ~inside;
        //}
    }

    cv::Mat output;
    cv::cvtColor(fill, output, CV_RGB2GRAY);

    return  output;
}

/* 获取无平面蒙版 */
cv::Mat GetNoPlaneMask(PointCloudPtr & cloud)
{
    /* 计算法向量 */
    PointCloudNTPtr normals(new pcl::PointCloud<PointNT>);
    normals->width = cloud->width; 
    normals->height = cloud->height;
    normals->resize(normals->width*normals->height);

    pcl::IntegralImageNormalEstimation<PointT, PointNT> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE );
    ne.setDepthDependentSmoothing(false);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    /* 利用法向量获取边缘 */
    cv::Mat edge = GetEdgeFromDepth(normals);
    OpenImage(edge, 1, 0, 0);
    
    /* 先尝试粗略修复 */
    RoughFixMask(cloud, edge);
    /* 尝试连接被意外分开的平面 */
    ConnectDepthMask(cloud, edge);

    /* 从边缘图形中找到不是背景的小平面块，输出无平面蒙版 */
    cv::Mat no_plane_mask = GetMaskFromEdge(cloud, normals, edge);
    OpenImage(no_plane_mask, 1, 0, 0);

    return no_plane_mask;
}

}
}


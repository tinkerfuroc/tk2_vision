//
// Created by 郭嘉丞 on 15/9/25.
//

#include "pcl_rebuild/ImageRebuild.h"
#include "pcl_rebuild/KinectParameters.h"
#include <iostream>

namespace tinker
{
namespace vision
{   
    using std::cerr;
    using std::endl;
    using std::cout;

    struct PixelNumber
    {
        int nx;
        int ny;
    };

    const int kLowResWidth = 512;
    const int kLowResHeight = 424;
    const int kHighResWidth = 1920;
    const int kHighResHeight = 1080;

    static PixelNumber GetPixelNumber(const pcl::PointXYZRGB &point, double projectionMat[3][4]);

    cv::Mat Get2DImageFromPointCloud(PointCloudPtr cloud)
    {
        cv::Mat rebuiltImage(kLowResHeight, kLowResWidth, CV_8UC3);
        for (int i = 0; i < rebuiltImage.rows; i++)
        {
            for (int j = 0; j < rebuiltImage.cols; j++)
            {
                rebuiltImage.at<cv::Vec3b>(i, j)[0] = 0;
                rebuiltImage.at<cv::Vec3b>(i, j)[1] = 0;
                rebuiltImage.at<cv::Vec3b>(i, j)[2] = 0;
            }
        }
        int minPixelX = INT32_MAX, maxPixelX = -1;
        int minPixelY = INT32_MAX, maxPixelY = -1;
        for (unsigned i=0; i<cloud->points.size(); i++)
        {
            const pcl::PointXYZRGB & point = cloud->points[i];
            PixelNumber pxNumber = GetPixelNumber(point, projectionParameter);
            if(pxNumber.nx >= kLowResWidth || pxNumber.nx < 0)
            {
    //            cerr << pxNumber.nx << " " << pxNumber.ny << endl;
                continue;
            }
            if(pxNumber.ny >= kLowResHeight || pxNumber.nx < 0)
            {
    //            cerr << pxNumber.nx << " " << pxNumber.ny << endl;
                continue;
            }
            cv::Vec3b &pixel = rebuiltImage.at<cv::Vec3b>(pxNumber.ny, pxNumber.nx);
            pixel[0] = point.b;
            pixel[1] = point.g;
            pixel[2] = point.r;
            if (pxNumber.nx > maxPixelX) maxPixelX = pxNumber.nx;
            if (pxNumber.nx < minPixelX) minPixelX = pxNumber.nx;
            if (pxNumber.ny > maxPixelY) maxPixelY = pxNumber.ny;
            if (pxNumber.ny < minPixelY) minPixelY = pxNumber.ny;
        }
        if(maxPixelX < 0 || minPixelX < 0) return cv::Mat();
        return cv::Mat(rebuiltImage,
                       cv::Range(minPixelY, maxPixelY+1),
                       cv::Range(minPixelX, maxPixelX+1));
    }

    cv::Mat GetHDImageFromPointCloud(PointCloudPtr cloud, cv::Mat &totalImage)
    {
        int minPixelX = INT32_MAX, maxPixelX = -1;
        int minPixelY = INT32_MAX, maxPixelY = -1;
        for (unsigned i=0; i<cloud->points.size(); i++)
        {
            const pcl::PointXYZRGB & point = cloud->points[i];
            PixelNumber pxNumber = GetPixelNumber(point, projectionParameter1080);
            if (pxNumber.nx > maxPixelX) maxPixelX = pxNumber.nx;
            if (pxNumber.nx < minPixelX) minPixelX = pxNumber.nx;
            if (pxNumber.ny > maxPixelY) maxPixelY = pxNumber.ny;
            if (pxNumber.ny < minPixelY) minPixelY = pxNumber.ny;
        }
        minPixelX = minPixelX < 0 ? 0 : minPixelX;
        maxPixelX = maxPixelX >= totalImage.cols ? totalImage.cols - 1 : maxPixelX;
        minPixelY = minPixelY < 0 ? 0 : minPixelY;
        maxPixelY = maxPixelY >= totalImage.cols ? totalImage.cols - 1 : maxPixelY;
        return cv::Mat(totalImage,
                       cv::Range(minPixelY, maxPixelY+1),
                       cv::Range(minPixelX, maxPixelX+1));
    }

    static PixelNumber GetPixelNumber(const pcl::PointXYZRGB &point, double projectionMat[3][4])
    {
        double projectedPoint[3];   //2d point in the homogeneous coordinate
        for(int i=0; i<3; i++)
        {
            projectedPoint[i] = projectionMat[i][0] * point.x +
                                projectionMat[i][1] * point.y +
                                projectionMat[i][2] * point.z +
                                projectionMat[i][3];
        }
        PixelNumber pixelNumber;
        pixelNumber.nx = (int)(projectedPoint[0] / projectedPoint[2]);
        pixelNumber.ny = (int)(projectedPoint[1] / projectedPoint[2]);
        return pixelNumber;
    }
}
}

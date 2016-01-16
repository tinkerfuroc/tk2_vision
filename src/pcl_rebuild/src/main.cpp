#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include "pcl_rebuild/PointCloudBuilder.h"
#include "pcl_rebuild/PointCloudUtilities.h"
#include "pcl_rebuild/ClusterDivider.h"
#include "pcl_rebuild/KinectParameters.h"
#include "pcl_rebuild/LineFilter.h"
#include "pcl_rebuild/PointCloudBuilder.h"
#include "pcl_rebuild/EntropyFilter.h"
#include "pcl_rebuild/ImageRebuild.h"
#include "pcl_rebuild/ColorChecker.h"

using namespace std;
using namespace tinker::vision;

void PrintDepth(cv::Mat depthMat);

void PrintPojectedXY(double x, double y, double z, double projectionMatrix[3][4]);

void PrintUsage();

int main(int argc, const char * argv[])
{
    if(argc != 5)
    {
        PrintUsage();
        return -1;
    }
    cv::Mat depth_mat = ReloadDepthImage(argv[1]);
    cv::Mat image_mat = cv::imread(argv[2]);
    cv::Mat hi_res_image = cv::imread(argv[3]);
   
    cv::Mat raw_depth_mat = ReloadDepthImage(argv[1]);
    cv::Mat raw_image_mat = cv::imread(argv[2]);
    int cnt_object_found=0;

    LineFilter line_filter;
    line_filter.Filter(depth_mat, image_mat);
    EntropyFilter entropy_filter(5, 0.4);
    entropy_filter.Filter(depth_mat, image_mat);

    PointCloudBuilder * builder = new PointCloudBuilder(depth_mat, image_mat);
    PointCloudPtr pointCloud = builder->GetPointCloud();
    //pcl::io::savePCDFile(argv[4], *pointCloud, true);
    IPointCloudDivider * divider = new ClusterDivider(pointCloud);
    vector<PointCloudPtr> divided_point_clouds = divider->GetDividedPointClouds();
    
    ColorChecker colorChecker(raw_image_mat);
 
    char buffer[20];
    for (int i = 0; i < (int) divided_point_clouds.size(); i++)
    {
        cv::Mat highResImg = GetHDImageFromPointCloud(divided_point_clouds[i], raw_image_mat);
        float cc = colorChecker.checkColor(highResImg);
        cout<<i<<"\t"<<cc<<endl;

        if (cc < 0.9)
        {
          string pcd_prefix = "dividedCloud";
          string lowRes_prefix = "lowRes";
          string highRes_prefix = "hiRes";
          ++cnt_object_found;
          sprintf(buffer, "%d", cnt_object_found);

          pcl::io::savePCDFile(pcd_prefix + buffer + ".pcd", *divided_point_clouds[i], true);
          cv::imwrite(lowRes_prefix + buffer + ".png", Get2DImageFromPointCloud(divided_point_clouds[i]));
          cv::imwrite(highRes_prefix + buffer + ".png", highResImg);
        }
         
    }
    delete divider;
	delete builder;
  
    cout << "Objects found:"  << cnt_object_found << endl;

    PointCloudBuilder * raw_builder = new PointCloudBuilder(raw_depth_mat, raw_image_mat);
    PointCloudPtr raw_pointCloud = raw_builder->GetPointCloud();
    pcl::io::savePCDFile(argv[4], *raw_pointCloud, true);  
    delete raw_builder;

    return 0;
}

void PrintDepth(cv::Mat depthMat)
{
    int x1 = 205, x2 = 377, y1 = 125, y2 = 229;
    cv::Mat mean;
    cv::Mat stddev;
    cv::Mat boardDepth(depthMat, cv::Range(y1, y2), cv::Range(x1, x2));
    cv::meanStdDev(boardDepth, mean, stddev);
    cout << mean.at<double>(0) << endl;
    cout << stddev.at<double>(0) << endl;
    cv::imshow("depth", boardDepth);
}

void PrintPojectedXY(double x, double y, double z, double projectionMatrix[3][4])
{
    double position[4] = {x, y, z, 1};
    double projectedPositions[3] = {0, 0, 0};
    cout << x  << " " << y << " " << z << endl;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            projectedPositions[i] += projectionMatrix[i][j] * position[j];
        }
    }
    cout << projectedPositions[0] / projectedPositions[2] << " "
    << projectedPositions[1] / projectedPositions[2] << endl;

}

void PrintUsage()
{
    cout << "KinectToPCL depthBinFile registeredImage rgbImage saveFile" << endl;
}

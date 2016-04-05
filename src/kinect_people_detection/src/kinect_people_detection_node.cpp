/*
 * Created by sugar10w, 2016.2.23
 * Last edited by sugar10w, 2016.2.24
 *
 * 在点云中找到人物 
 * 在ROS上连续地发出人物位置信息
 *  
 * 使用libfreenect2连接Kinect2
 * 使用GroundBasedPeopleDetectionApp查找人物
 *   
 * TODO 注意，默认了Kinect距离地面的高度为1.10米，且视线平行于地面
 * 
 */



#include "pcl_rebuild/people_detection.h"

#include <signal.h>
#include <cstdio>
#include <cstdlib>

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/people/ground_based_people_detection_app.h>

#include <pcl_rebuild/PointCloudBuilder.h>

//ROS
#include <ros/ros.h>
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
#include <sensor_msgs/PointCloud.h>

typedef pcl::PointXYZRGB PointT;

using std::cout;
using std::endl;
using namespace tinker::vision;
using namespace libfreenect2;

// kinect距离地面的高度，单位米 
const float KINECT_HEIGHT = 1.10;

// 显示点云的窗口 PCD Viewer
//pcl::visualization::PCLVisualizer viewer("PCD Viewer");

// 全局控制开关
bool protonect_shutdown = false; 

// 处理SIGNIT(InterruptKey)信号，停止显示
void sigint_handler(int signal_number)
{
  protonect_shutdown = true;
}


int main(int argc, char *argv[])
{

  // --------------------------------------------------------------------------
  // ROS 初始化
  ros::init(argc, argv, "kinect_people_detection_node");
  ros::NodeHandle node_handle;
  ros::Publisher people_pub_ = node_handle.advertise<people_msgs::People>("kinect_found_people", 1);
  ros::Rate loop_rate(10);
  unsigned int seq_ = 0;

  // --------------------------------------------------------------------------
  // 对人物识别的初始化


  float min_height = 1.50,
        max_height = 2.00,
        min_width = 0.10,
        max_width = 2.00;
  float voxel_size = 0.06; 
  Eigen::Matrix3f rgb_intrinsics_matrix; // TODO: Kinect RGB camera intrinsics; 有何作用? 对Kinect2是否必要?
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0;   

  //TODO 设置地面的方程 y - KINECT_HEIGHT = 0
  Eigen::VectorXf ground_coeffs;  
  ground_coeffs.resize(4);
  ground_coeffs(0) = 0;
  ground_coeffs(1) = 1;
  ground_coeffs(2) = 0;
  ground_coeffs(3) = -KINECT_HEIGHT;
  
  // Create classifier for people detection:  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  std::string svm_filename = "../data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";  //TODO 更新此文件; 原训练文件暂时无效;
  float min_confidence = -1.5;                                                          //TODO 并修正置信度阈值
  person_classifier.loadSVMFromFile(svm_filename);  

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setPersonClusterLimits(min_height, max_height, min_width, max_width);     // set person size limits

  // --------------------------------------------------------------------------
  // 配置 Kinect2

  // 创建一个Debug级别的Logger
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  // 初始化对象
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  std::string serial;
  // 查找Kinect2设备
  if(freenect2.enumerateDevices() == 0)
  { std::cout << "no device connected!" << std::endl; return -1; }
  serial = freenect2.getDefaultDeviceSerialNumber();
  // 打开Kinect2设备
  dev = freenect2.openDevice(serial);
  if(dev == 0)
  { std::cout << "failure opening device!" << std::endl; return -1; }

  // 改绑SIGINT(InterruptKey)信号，用于安全退出
  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

  // 配置Kinect2的监听对象，用于获取总数据帧
  //   Color(1920*1080 32-bit BGRX)
  //   Ir(512*424 float)
  //   Depth(512*424 float mm)
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;
  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

  // 开始运行
  dev->start();
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  // 配置校准器registration (使用默认参数)
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  // --------------------------------------------------------------------------
  // 开始循环
  while(ros::ok() && !protonect_shutdown)
  {

    // 等待接收总数据帧
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    // 校准
    registration->apply(rgb, depth, &undistorted, &registered);

    // 选取depth
    cv::Mat depthMatrix(depth->height, depth->width, CV_32FC1, depth->data);
    // 选取rgb
    cv::Mat rgbMatrix(rgb->height, rgb->width , CV_8UC4, rgb->data);
    // 选取registered
    cv::Mat registeredMatrix(registered.height, registered.width, CV_8UC4, registered.data);


    // 预处理 depth, rgb, registered
    depthMatrix /= 4500.0f;
    cv::cvtColor(rgbMatrix, rgbMatrix, CV_BGRA2BGR);
    cv::cvtColor(registeredMatrix, registeredMatrix, CV_BGRA2BGR);
    // 获取点云cloud
    PointCloudBuilder builder(depthMatrix, registeredMatrix);
    PointCloudPtr cloud = builder.GetPointCloud();
    // 预处理点云cloud
    for (int i=0; i<cloud->size(); ++i)
    {
      PointT & point = cloud->points[i];
      point.x *= 0.01;
      point.y *= -0.01;
      point.z *= 0.01;
    }
    

    // 在点云上应用people_detection
    std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
    people_detector.setInputCloud(cloud);
    people_detector.setGround(ground_coeffs);                    // set floor coefficients
    people_detector.compute(clusters);                           // perform people detection

    // 输出数据
    people_msgs::People people;
    people.header.seq = seq_++;
    people.header.stamp = ros::Time::now();
    // people.header.frame_id = frame_id_;

    unsigned int k = 0;
    for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
        if (it->getHeight() > min_height)
        {
          k++;
          Eigen::Vector3f point_eigen3f = it->getTCenter();
          cv::Point3f point_cv3f(point_eigen3f[0]*100, -point_eigen3f[1]*100, point_eigen3f[2]*100);
          
          people_msgs::Person person;
          person.position.x = point_cv3f.z; // 注意：转移到机器人坐标
          person.position.y = point_cv3f.x;
          person.position.z = 0;
          people.people.push_back(person);

        } 
    ROS_INFO("%d people found",k);

    people_pub_.publish(people);
    ros::spinOnce();
    loop_rate.sleep();


    // 释放总数据帧
    listener.release(frames);
  }

  // 关闭设备
  dev->stop();
  dev->close();

  delete registration;

}


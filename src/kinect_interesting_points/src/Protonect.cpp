/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/** @file Protonect.cpp Main application file. */


#include <sys/time.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include <iostream>
#include <fstream>
#include <string>


#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>

#include "pcl_rebuild/PointCloudBuilder.h"
#include "pcl_rebuild/ClusterDivider.h"
#include "pcl_rebuild/LineFilter.h"
#include "pcl_rebuild/EntropyFilter.h"
#include "pcl_rebuild/ColorChecker.h"
#include "pcl_rebuild/ImageRebuild.h"

#include "pcl_rebuild/Protonect.h"

namespace tinker{
namespace vision{

using std::cout;
using std::endl;
using namespace libfreenect2;

static bool protonect_shutdown = false; ///< Whether the running application should shut down.

static void sigint_handler(int s)
{
  protonect_shutdown = true;
}

//The following demostrates how to create a custom logger
// logger

class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
   {
    if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
  bool good()
   {
    return logfile_.is_open() && logfile_.good();
  }
  virtual void log(Level level, const std::string &message)
   {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};

static void delay_ms(int time_ms)
{
  struct timeval t_start, t_now;
  int cnt, last_cnt;
  gettimeofday(&t_start, NULL);
  while (true)
  {
    gettimeofday(&t_now, NULL);
    cnt = ( t_now.tv_sec - t_start.tv_sec )*1000 + t_now.tv_usec/1000 - t_start.tv_usec/1000;
    if (cnt>time_ms) break;
  }
}

static cv::Point3f getWeightPoint(PointCloudPtr cloud)
{
  float sX=0, sY=0, sZ=0;
  int size= cloud->width*cloud->height;

  for (int i=0; i<size; ++i)
  {
    pcl::PointXYZRGB point = cloud->points[i];
    sX+=point.x; sY+=point.y; sZ+=point.z;
  }

  
  return cv::Point3f(sX/size, sY/size, sZ/size);
  
}

//int main(int argc, char *argv[])
void getInterestingPoints(std::vector<cv::Point3f> & points)
{ 

  // create a console logger with debug level (default is console logger with info level)
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));

  MyFileLogger *filelogger = new MyFileLogger(getenv("LOGFILE"));
  if (filelogger->good())
    libfreenect2::setGlobalLogger(filelogger);

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  if(freenect2.enumerateDevices() == 0)
   {
    std::cout << "no device connected!" << std::endl;
    return ;
  }
  std::string serial = freenect2.getDefaultDeviceSerialNumber();


    dev = freenect2.openDevice(serial);

  if(dev == 0)
   {
    std::cout << "failure opening device!" << std::endl;
    return ;
  }

  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

// listeners
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

// start
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

// registration setup
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

//---------------------------------------------------------------
  int framecount=0; 

// loop start  
  
//while(!protonect_shutdown)
//{
    
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

// registration
    registration->apply(rgb, depth, &undistorted, &registered);
        
    framecount++;
    std::cout<<"["<<framecount<<"]"<<std::endl;

    cv::Mat depthMatrix(depth->height, depth->width, CV_32FC1, depth->data);
    depthMatrix /= 4500.0f;
    cv::Mat rgbMatrix(rgb->height, rgb->width , CV_8UC4, rgb->data);    
    cv::Mat display(720, 1280, CV_8UC3);
    cv::resize(rgbMatrix, display, display.size());


    cv::Mat registeredMatrix(registered.height, registered.width, CV_8UC4, registered.data);

    LineFilter line_filter;
    line_filter.Filter(depthMatrix, registeredMatrix);
    EntropyFilter entropy_filter(5, 0.4);
    entropy_filter.Filter(depthMatrix, registeredMatrix);
      
    PointCloudBuilder * builder = new PointCloudBuilder(depthMatrix, registeredMatrix);
    PointCloudPtr pointCloud = builder->GetPointCloud();
    IPointCloudDivider * divider = new ClusterDivider(pointCloud);
    std::vector<PointCloudPtr> divided_point_clouds = divider->GetDividedPointClouds();

    listener.release(frames);
  
    points.clear();
    for (int i=0; i<(int)divided_point_clouds.size(); ++i)
    {
      points.push_back(getWeightPoint(divided_point_clouds[i]));
    }

//}

// stop
  dev->stop();
  dev->close();

  delete registration;

}

}
}

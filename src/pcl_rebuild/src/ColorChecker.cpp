
#include"pcl_rebuild/ColorChecker.h"
#include<iostream>

namespace tinker
{ 
namespace vision
{

ColorChecker::ColorChecker(cv::Mat & rawImage)
{
  int colorRange[360]={0};
  int maxH=-720, cntMaxH=-1;
  cv::Mat img_hsv;
  cv::cvtColor(rawImage, img_hsv, CV_BGR2HSV);
  for (int y=0; y<img_hsv.rows; ++y)
  {
    for (int x=0; x<img_hsv.cols; ++x)
    {
      int h = img_hsv.at<cv::Vec3b>(y,x)[0] * 2;
      if (h==0) continue;  // `h==0` might indicate the failure in hsv conversion
      colorRange[ h           ] +=2;
      colorRange[ (h+359)%360 ] +=1;
      colorRange[ (h+1  )%360 ] +=1;
    }
  }
  
  for (int i=0; i<360; ++i)
    if (colorRange[i]>cntMaxH) 
    {
      cntMaxH=colorRange[i]; 
      maxH=i;
    }
  
  _maxColorH=maxH;
} 

float ColorChecker::checkColor(cv::Mat & img, int threshold)
{
  int cntBkColorP=0;
  cv::Mat img_hsv;
  cv::cvtColor(img, img_hsv, CV_BGR2HSV);

  for (int y=0; y<img_hsv.rows; ++y)
  {
    for (int x=0; x<img_hsv.cols; ++x)
    {
      int h = img_hsv.at<cv::Vec3b>(y,x)[0] * 2;
      if (h==0) // considered to be meaningless point
      {
         ++cntBkColorP;
         continue; 
      } 

      if (abs(h-_maxColorH)<=threshold || abs(h+360-_maxColorH)<=threshold || abs(h-360-_maxColorH)<=threshold ) 
      { 
        ++cntBkColorP; 
        //img.at<cv::Vec3b>(y,x)[0]=0;
        //img.at<cv::Vec3b>(y,x)[1]=0;
        //img.at<cv::Vec3b>(y,x)[2]=0;
      }
    }
  }
  return (float)cntBkColorP/(float)(img_hsv.rows*img_hsv.cols);
}


}
}



#include"pcl_rebuild/ColorChecker.h"
#include<iostream>

namespace tinker
{ 
namespace vision
{

using std::acos;
using std::sqrt;
using std::abs;

static int getColorHfromRGB(int r, int g, int b)
{
  if ((r-g)*(r-g)+(r-b)*(g-b)==0) return -1;
  double theta = acos( (2*r-g-b)/2/sqrt((r-g)*(r-g)+(r-b)*(g-b)))*180/3.14;
  if (b<=g) return theta;
  else return 360-theta;
} 

static int getColorHfromMatYX(cv::Mat &img, int y, int x)
{ 
   return getColorHfromRGB(
         img.at<cv::Vec3b>(y,x)[0],
         img.at<cv::Vec3b>(y,x)[1],
         img.at<cv::Vec3b>(y,x)[2] 
       );
}

ColorChecker::ColorChecker(cv::Mat & rawImage)
{
  int colorRange[360]={0};
  int maxH=-720, cntMaxH=-1;
  for (int y=0; y<rawImage.rows; ++y)
  {
    for (int x=0; x<rawImage.cols; ++x)
    {
      int h= getColorHfromMatYX(rawImage, y, x);
      if (h<0) continue; else h%=360;
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
  int cntSimilar=0;
  for (int y=0; y<img.rows; ++y)
  {
    for (int x=0; x<img.cols; ++x)
    {
      int h= getColorHfromMatYX(img, y, x);
      if (h<0) 
      {
         ++cntSimilar;
         continue; 
      } else h%=360;

      if (abs(h-_maxColorH)<=threshold || abs(h+360-_maxColorH)<=threshold || abs(h-360-_maxColorH)<=threshold ) 
      { 
        cntSimilar++; 
        //img.at<cv::Vec3b>(y,x)[0]=0;
        //img.at<cv::Vec3b>(y,x)[1]=0;
        //img.at<cv::Vec3b>(y,x)[2]=0;
      }
    }
  }
  return (float)cntSimilar/(float)(img.rows*img.cols);
}


}
}


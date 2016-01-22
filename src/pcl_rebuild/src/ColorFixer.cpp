#include"pcl_rebuild/ColorFixer.h"

namespace tinker{
namespace vision{

static bool isUselessPoint(cv::Vec3b point)
{
  return point[0]+point[1]+point[2]==0 
        || point[0]==255&&point[1]==255&&point[2]==255 ;
}

cv::Mat fixColor(cv::Mat img, int RANGE)
{
  cv::Mat img2 = cv::Mat(img.rows, img.cols, CV_8UC3);
  for (int y=0; y<img.rows; ++y)
    for (int x=0; x<img.cols; ++x)
    {
      img2.at<cv::Vec3b>(y,x)[0]=img.at<cv::Vec3b>(y,x)[0];
      img2.at<cv::Vec3b>(y,x)[1]=img.at<cv::Vec3b>(y,x)[1];
      img2.at<cv::Vec3b>(y,x)[2]=img.at<cv::Vec3b>(y,x)[2];      
    }
  for (int y=RANGE; y<img.rows-RANGE; ++y)
    for (int x=RANGE; x<img.cols-RANGE; ++x)
      if ( isUselessPoint(img.at<cv::Vec3b>(y,x)))
      {
         int b=0, g=0, r=0; int cnt=0;
         for (int i=y-RANGE; i<=y+RANGE; ++i)
           for (int j=x-RANGE; j<=x+RANGE; ++j)
            if ( ! isUselessPoint(img.at<cv::Vec3b>(i,j)))
            {
              b+=img.at<cv::Vec3b>(i,j)[0]; g+=img.at<cv::Vec3b>(i,j)[1]; r+=img.at<cv::Vec3b>(i,j)[2]; ++cnt;
             }
         if (cnt>=RANGE*RANGE)
         {
           img2.at<cv::Vec3b>(y,x)[0]=b/cnt; img2.at<cv::Vec3b>(y,x)[1]=g/cnt; img2.at<cv::Vec3b>(y,x)[2]=r/cnt;
         }
         else
         {
           img2.at<cv::Vec3b>(y,x)[0]=0; img2.at<cv::Vec3b>(y,x)[1]=0; img2.at<cv::Vec3b>(y,x)[2]=0;
         }
       }
  return img2;
}

}
}

#include "tinker_object_recognition/graph_filter/ColorFixer.h"

namespace tinker{
namespace vision{

static bool IsUselessPoint(cv::Vec3b point)
{
  return point[0]+point[1]+point[2]==0 
        || point[0]==255&&point[1]==255&&point[2]==255 ;
}

void FixColor(cv::Mat &img, const int kRange)
{
    for (int y=kRange; y<img.rows-kRange; ++y) {
        for (int x=kRange; x<img.cols-kRange; ++x) {
            if ( IsUselessPoint(img.at<cv::Vec3b>(y,x))) {
                int b=0, g=0, r=0; int cnt=0;
                for (int i=y-kRange; i<=y+kRange; ++i) {
                    for (int j=x-kRange; j<=x+kRange; ++j) {
                        if (!IsUselessPoint(img.at<cv::Vec3b>(i,j))) {
                        b+=img.at<cv::Vec3b>(i,j)[0]; 
                        g+=img.at<cv::Vec3b>(i,j)[1]; 
                        r+=img.at<cv::Vec3b>(i,j)[2]; 
                        ++cnt;
                        }
                    }
                }
                if (cnt>=kRange*kRange) {
                    img.at<cv::Vec3b>(y,x)[0]=b/cnt; 
                    img.at<cv::Vec3b>(y,x)[1]=g/cnt; 
                    img.at<cv::Vec3b>(y,x)[2]=r/cnt;
                }
                else {
                    img.at<cv::Vec3b>(y,x)[0]=0; 
                    img.at<cv::Vec3b>(y,x)[1]=0;
                    img.at<cv::Vec3b>(y,x)[2]=0;
                }
            }
        }
    }
}

}
}

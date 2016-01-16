//sugar10w, 2016.1.16

#ifndef _OBJECTFINDER_COLORCHECKER_H_
#define _OBJECTFINDER_COLORCHECKER_H_

#include<cmath>
#include<opencv2/opencv.hpp>

namespace tinker{ 
namespace vision{

static int getColorHfromRGB(int r, int g, int b);
static int getColorHfromMatYX(cv::Mat &img, int y, int x);

class ColorChecker
 {
public:
  ColorChecker(cv::Mat & rawImage);
  float checkColor(cv::Mat & img, int threshold=14);
private:
  int _maxColorH;
};


}
}
#endif // _OBJECTFINDER_COLORCHECKER_H_

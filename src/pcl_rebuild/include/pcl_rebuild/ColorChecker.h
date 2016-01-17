//sugar10w, 2016.1.16

#ifndef _OBJECTFINDER_COLORCHECKER_H_
#define _OBJECTFINDER_COLORCHECKER_H_

#include<opencv2/opencv.hpp>

namespace tinker{ 
namespace vision{

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

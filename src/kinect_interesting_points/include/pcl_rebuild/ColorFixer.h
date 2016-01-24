
#ifndef _OBJECTFINDER_COLORFIXER_
#define _OBJECTFINDER_COLORFIXER_

#include<opencv2/opencv.hpp>

namespace tinker{
namespace vision{

cv::Mat fixColor(cv::Mat img, int RANGE=2);


}
}

#endif // _OBJECTFINDER_COLORFIXER_


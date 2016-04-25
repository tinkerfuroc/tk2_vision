
#ifndef _OBJECTFINDER_COLORFIXER_
#define _OBJECTFINDER_COLORFIXER_

#include <opencv2/opencv.hpp>

namespace tinker{
namespace vision{

void FixColor(cv::Mat &img, const int kRange=2);


}
}

#endif // _OBJECTFINDER_COLORFIXER_


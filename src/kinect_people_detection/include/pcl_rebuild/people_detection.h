
#ifndef _KINECT_PEOPLE_DETECTION_
#define _KINECT_PEOPLE_DETECTION_

#include<opencv2/opencv.hpp>
#include<vector>


namespace tinker{
namespace vision{



void getPeoplePosition(std::vector<cv::Point3f> & points);

}
}

#endif //_KINECT_PEOPLE_DETECTION_

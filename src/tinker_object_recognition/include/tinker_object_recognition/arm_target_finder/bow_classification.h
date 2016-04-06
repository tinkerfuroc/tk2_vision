#ifndef __TINKER_BOW_CLASSIFICATION_H__
#define __TINKER_BOW_CLASSIFICATION_H__

#include "tinker_object_recognition/common.h"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/ml/ml.hpp"
#include <map>
#include <string>
#include <vector>

namespace tinker {
namespace vision {

cv::Mat trainVocabulary(const std::map<std::vector<std::string> > & samplefiles,
        const cv::Ptr<FeatureDetector>& fdetector, 
        const cv::Ptr<DescriptorExtractor>& dextractor);

}
}

#endif


#ifndef __TINKER_VISION_BOW_PARAMETER_H__
#define __TINKER_VISION_BOW_PARAMETER_H__

#include "tinker_object_recognition/common.h"
#include "opencv2/opencv.hpp"
#include <string>


namespace tinker {
namespace vision {

struct VocabTrainParams {
    VocabTrainParams()
        : trainObjClass(""),
          vocabSize(100),
          memoryUse(200),
          descProportion(0.3f) {}
    VocabTrainParams(const std::string _trainObjClass, size_t _vocabSize,
                     size_t _memoryUse, float _descProportion)
        : trainObjClass(_trainObjClass),
          vocabSize((int)_vocabSize),
          memoryUse((int)_memoryUse),
          descProportion(_descProportion) {}
    void read(const cv::FileNode& fn) {
        fn["trainObjClass"] >> trainObjClass;
        fn["vocabSize"] >> vocabSize;
        fn["memoryUse"] >> memoryUse;
        fn["descProportion"] >> descProportion;
    }
    void write(cv::FileStorage& fs) const {
        fs << "trainObjClass" << trainObjClass;
        fs << "vocabSize" << vocabSize;
        fs << "memoryUse" << memoryUse;
        fs << "descProportion" << descProportion;
    }

    std::string trainObjClass;  // Object class used for training visual vocabulary.
    int vocabSize;  // number of visual words in vocabulary to train
    int memoryUse;  // Memory to preallocate (in MB) when training vocab.
    // Change this depending on the size of the dataset/available memory.
    float descProportion;  // Specifies the number of descriptors to use from
                           // each image as a proportion of the total num descs.
};

struct SVMTrainParamsExt {
    SVMTrainParamsExt()
        : descPercent(1.0f), targetRatio(0.4f), balanceClasses(true) {}
    SVMTrainParamsExt(float _descPercent, float _targetRatio,
                      bool _balanceClasses)
        : descPercent(_descPercent),
          targetRatio(_targetRatio),
          balanceClasses(_balanceClasses) {}
    void read(const cv::FileNode& fn) {
        fn["descPercent"] >> descPercent;
        fn["targetRatio"] >> targetRatio;
        fn["balanceClasses"] >> balanceClasses;
    }
    void write(cv::FileStorage& fs) const {
        fs << "descPercent" << descPercent;
        fs << "targetRatio" << targetRatio;
        fs << "balanceClasses" << balanceClasses;
    }
    float descPercent;  // Percentage of extracted descriptors to use for
                        // training.
    float targetRatio;  // Try to get this ratio of positive to negative samples
                        // (minimum).
    bool balanceClasses;  // Balance class weights by number of samples in each
                          // (if true cSvmTrainTargetRatio is ignored).
};


}
}

#endif

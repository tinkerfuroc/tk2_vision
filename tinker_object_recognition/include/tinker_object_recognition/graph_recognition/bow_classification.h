#ifndef __TINKER_VISION_BOW_RECOGNITION_H__
#define __TINKER_VISION_BOW_RECOGNITION_H__

#include "tinker_object_recognition/common.h"
#include "opencv2/opencv.hpp"
#include "opencv2/ml/ml.hpp"

#include <map>
#include <string>
#include <vector>
#include <object_recognition_msgs/RecognizedObject.h>
#include <ros/ros.h>
#include "tinker_object_recognition/graph_recognition/bow_data.h"
#include "tinker_object_recognition/graph_recognition/bow_parameter.h"

namespace tinker {
namespace vision {

class BoWRecognition {
public:
    BoWRecognition() {}
    BoWRecognition(XmlRpc::XmlRpcValue & image_class_info);
    bool SaveModelFile();
    cv::Mat TrainVocabulary();
    //Caution: Train or load vocabulary first before you do anythin else!
    bool WriteVocabulary(const std::string & filename, const cv::Mat & vocabulary);
    cv::Mat ReadVocabulary(const std::string &filename);
    void TrainSVMClassifier(CvSVM &svm, const std::string & class_name);
    void WriteSVMClassifer(CvSVM &svm, const std::string & class_name);
    void ReadSVMClassifier(CvSVM &svm, const std::string & class_name);
    std::vector<std::string> GetObjectClasses() {
        return bow_data_.GetObjectClasses();
    }

    void CalculateImageDescriptor(const cv::Mat &image, cv::Mat &descriptor);

private:
    std::vector<cv::Mat> CalculateImageDescriptors(std::vector<ObdImage> & images);
    void RemoveEmptyBowImageDescriptors(std::vector<ObdImage>& images,
                               std::vector<cv::Mat>& bowImageDescriptors,
                               std::vector<char>& objectPresent);
    void SetSVMParams(CvSVMParams& svmParams, CvMat& class_wts_cv,
                     const cv::Mat& responses, bool balanceClasses);
    BoWData bow_data_;
    VocabTrainParams vocab_params_;
    SVMTrainParamsExt svm_params_;
    std::string matcher_type_;
    std::string image_folder_name;
    std::string vocabulary_filename_;
    std::map<std::string, std::vector<std::string> > image_class_filenames_;
    cv::Ptr<cv::FeatureDetector> feature_detector_;
    cv::Ptr<cv::DescriptorExtractor> desc_extractor_;
    cv::Ptr<cv::DescriptorMatcher> desc_matcher_;
    cv::Ptr<cv::BOWImgDescriptorExtractor> bow_extractor_;
};

}
}

#endif


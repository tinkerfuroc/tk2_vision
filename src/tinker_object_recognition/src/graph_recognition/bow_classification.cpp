#include "tinker_object_recognition/graph_recognition/bow_classification.h"
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <cstdlib>
#include "tinker_object_recognition/common.h"
#include <opencv2/nonfree/nonfree.hpp>

using std::string;
using std::map;
using std::vector;

namespace tinker {
namespace vision {
BoWRecognition::BoWRecognition(XmlRpc::XmlRpcValue& image_class_info)
    : bow_data_(image_class_info),
      descriptor_type_("SURF"),
      matcher_type_("BruteForce") {
    ROS_INFO("BoW extractor initializing");
    cv::initModule_nonfree();
    if (image_class_info.hasMember("descriptor_type"))
        descriptor_type_ = (string)image_class_info["descriptor_type"];
    if (image_class_info.hasMember("matcher_type"))
        matcher_type_ = (string)image_class_info["matcher_type"];
    ROS_INFO("Using %s %s", descriptor_type_.c_str(), matcher_type_.c_str());
    ROS_ASSERT(image_class_info.hasMember("image_folder_name"));
    image_folder_name = (string)image_class_info["image_folder_name"];
    feature_detector_ = cv::FeatureDetector::create(descriptor_type_);
    desc_extractor_ = cv::DescriptorExtractor::create(descriptor_type_);
    desc_matcher_ = cv::DescriptorMatcher::create(matcher_type_);
    ROS_ASSERT(!feature_detector_.empty());
    ROS_ASSERT(!desc_extractor_.empty());
    ROS_ASSERT(!desc_matcher_.empty());
    bow_extractor_ = cv::Ptr<cv::BOWImgDescriptorExtractor>(
        new cv::BOWImgDescriptorExtractor(desc_extractor_, desc_matcher_));
    ROS_INFO("BoW extractor initialization complete");
}

bool BoWRecognition::SaveModelFile() { return true; }

cv::Mat BoWRecognition::TrainVocabulary() {
    cv::Mat vocabulary;
    ROS_ASSERT(desc_extractor_->descriptorType() == CV_32FC1);
    const int elemSize = CV_ELEM_SIZE(desc_extractor_->descriptorType());
    const int descByteSize = desc_extractor_->descriptorSize() * elemSize;
    const int bytesInMB = 1048576;
    const int maxDescCount =
        (vocab_params_.memoryUse * bytesInMB) /
        descByteSize;  // Total number of descs to use for training.

    ROS_INFO("Extracting Data...");
    vector<ObdImage> images;
    vector<char> objectPresent;
    bow_data_.GetClassImages(vocab_params_.trainObjClass, CV_OBD_TRAIN, images,
                             objectPresent);

    ROS_INFO("Computing Descriptors...");
    cv::RNG& rng = cv::theRNG();
    cv::TermCriteria terminate_criterion;
    terminate_criterion.epsilon = FLT_EPSILON;
    cv::BOWKMeansTrainer bowTrainer(
        vocab_params_.vocabSize, terminate_criterion, 3, cv::KMEANS_PP_CENTERS);

    while (images.size() > 0) {
        if (bowTrainer.descripotorsCount() > maxDescCount) {
            break;
        }

        // Randomly pick and compute the descriptors from that image.
        int randImgIdx = rng((unsigned)images.size());
        cv::Mat colorImage = cv::imread(images[randImgIdx].path);
        vector<cv::KeyPoint> imageKeypoints;
        feature_detector_->detect(colorImage, imageKeypoints);
        cv::Mat imageDescriptors;
        desc_extractor_->compute(colorImage, imageKeypoints, imageDescriptors);

        if (!imageDescriptors.empty()) {
            int descCount = imageDescriptors.rows;
            // Extract trainParams.descProportion descriptors from the
            // image, breaking if the 'allDescriptors' matrix becomes full
            int descsToExtract = static_cast<int>(
                vocab_params_.descProportion * static_cast<float>(descCount));
            // Fill mask of used descriptors
            vector<char> usedMask(descCount, false);
            fill(usedMask.begin(), usedMask.begin() + descsToExtract, true);
            for (int i = 0; i < descCount; i++) {
                int i1 = rng(descCount), i2 = rng(descCount);
                char tmp = usedMask[i1];
                usedMask[i1] = usedMask[i2];
                usedMask[i2] = tmp;
            }

            for (int i = 0; i < descCount; i++) {
                if (usedMask[i] &&
                    bowTrainer.descripotorsCount() < maxDescCount)
                    bowTrainer.add(imageDescriptors.row(i));
            }
        }

        // Delete the current element from images so it is not added again
        images.erase(images.begin() + randImgIdx);
    }

    ROS_INFO("Get %d descriptors", bowTrainer.descripotorsCount());

    ROS_INFO("Training vocabulary...");
    vocabulary = bowTrainer.cluster();
    bow_extractor_->setVocabulary(vocabulary);
    return vocabulary;
}

bool BoWRecognition::WriteVocabulary(const string& filename,
                                     const cv::Mat& vocabulary) {
    ROS_INFO("Saving vocabulary...");
    cv::FileStorage fs(image_folder_name + "/" + filename, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "vocabulary" << vocabulary;
        return true;
    }
    return false;
}

cv::Mat BoWRecognition::ReadVocabulary(const string& filename) {
    cv::Mat vocabulary;
    ROS_INFO("Reading vocabulary...");
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["vocabulary"] >> vocabulary;
        ROS_INFO("done");
        bow_extractor_->setVocabulary(vocabulary);
        return vocabulary;
    }
    ROS_ERROR("Failed to read vocabulary");
    exit(-1);
    return cv::Mat();
}

void BoWRecognition::TrainSVMClassifier(CvSVM& svm, const string& class_name) {
    ROS_INFO("Training svm for %s", class_name.c_str());
    // Get classification ground truth for images in the training set
    vector<ObdImage> images;
    vector<cv::Mat> bowImageDescriptors;
    vector<char> objectPresent;
    bow_data_.GetClassImages(class_name, CV_OBD_TEST, images, objectPresent);

    // Compute the bag of words vector for each image in the training set.
    bowImageDescriptors = CalculateImageDescriptors(images);

    // Remove any images for which descriptors could not be calculated
    RemoveEmptyBowImageDescriptors(images, bowImageDescriptors, objectPresent);

    // Prepare the input matrices for SVM training.
    cv::Mat trainData((int)images.size(), bow_extractor_->getVocabulary().rows,
                      CV_32FC1);
    cv::Mat responses((int)images.size(), 1, CV_32SC1);
    // Transfer bag of words vectors and responses across to the training
    // data matrices
    for (size_t imageIdx = 0; imageIdx < images.size(); imageIdx++) {
        // Transfer image descriptor (bag of words vector) to training data
        // matrix
        cv::Mat submat = trainData.row((int)imageIdx);
        if (bowImageDescriptors[imageIdx].cols !=
            bow_extractor_->descriptorSize()) {
            ROS_ERROR("BoW image descriptor size doesn't match");
            exit(-1);
        }
        bowImageDescriptors[imageIdx].copyTo(submat);
        // Set response value
        responses.at<int>((int)imageIdx) = objectPresent[imageIdx] ? 1 : -1;
    }
    CvSVMParams svmParams;
    CvMat class_wts_cv;
    SetSVMParams(svmParams, class_wts_cv, responses, false);
    svm.train_auto(trainData, responses, cv::Mat(), cv::Mat(), svmParams);
    ROS_INFO("Train complete");
}

void BoWRecognition::WriteSVMClassifer(CvSVM& svm,
                                       const std::string& class_name) {
    string svmFilename = image_folder_name + "/" + class_name + ".xml.gz";
    svm.save(svmFilename.c_str());
    ROS_INFO("SVM for %s written to %s", class_name.c_str(),
             svmFilename.c_str());
}

void BoWRecognition::ReadSVMClassifier(CvSVM& svm, const string& class_name) {
    string svmFilename = image_folder_name + "/" + class_name + ".xml.gz";
    cv::FileStorage fs(svmFilename, cv::FileStorage::READ);
    if (fs.isOpened()) {
        svm.load(svmFilename.c_str());
        ROS_INFO("Loaded svm file for %s", class_name.c_str());
    } else {
        ROS_ERROR("Failed to load svm file for %s", class_name.c_str());
    }
}

std::vector<cv::Mat> BoWRecognition::CalculateImageDescriptors(
    std::vector<ObdImage>& images) {
    ROS_ASSERT(!bow_extractor_->getVocabulary().empty());
    vector<cv::Mat> imageDescriptors;
    imageDescriptors.resize(images.size());
    for (size_t i = 0; i < images.size(); i++) {
        cv::Mat colorImage = cv::imread(images[i].path);
        CalculateImageDescriptor(colorImage, imageDescriptors[i]);
    }
    return imageDescriptors;
}

void BoWRecognition::CalculateImageDescriptor(const cv::Mat &image, cv::Mat & descriptor) {
    vector<cv::KeyPoint> keypoints;
    feature_detector_->detect(image, keypoints);
    bow_extractor_->compute(image, keypoints, descriptor);
}


void BoWRecognition::RemoveEmptyBowImageDescriptors(
    std::vector<ObdImage>& images, std::vector<cv::Mat>& bowImageDescriptors,
    std::vector<char>& objectPresent) {
    ROS_ASSERT(!images.empty());
    for (int i = (int)images.size() - 1; i >= 0; i--) {
        bool res = bowImageDescriptors[i].empty();
        if (res) {
            images.erase(images.begin() + i);
            bowImageDescriptors.erase(bowImageDescriptors.begin() + i);
            objectPresent.erase(objectPresent.begin() + i);
        }
    }
}

void BoWRecognition::SetSVMParams(CvSVMParams& svmParams, CvMat& class_wts_cv,
                                  const cv::Mat& responses,
                                  bool balanceClasses) {
    int pos_ex = countNonZero(responses == 1);
    int neg_ex = countNonZero(responses == -1);
    ROS_INFO("%d positive, %d negative", pos_ex, neg_ex);

    svmParams.svm_type = CvSVM::C_SVC;
    svmParams.kernel_type = CvSVM::RBF;
    if (balanceClasses) {
        cv::Mat class_wts(2, 1, CV_32FC1);
        // The first training sample determines the '+1' class internally, even
        // if it is negative,
        // so store whether this is the case so that the class weights can be
        // reversed accordingly.
        bool reversed_classes = (responses.at<float>(0) < 0.f);
        if (reversed_classes == false) {
            class_wts.at<float>(0) =
                static_cast<float>(pos_ex) /
                static_cast<float>(pos_ex + neg_ex);  // weighting for costs of
                                                      // positive class + 1
                                                      // (i.e. cost of false
                                                      // positive - larger gives
                                                      // greater cost)
            class_wts.at<float>(1) =
                static_cast<float>(neg_ex) /
                static_cast<float>(pos_ex + neg_ex);  // weighting for costs of
                                                      // negative class - 1
                                                      // (i.e. cost of false
                                                      // negative)
        } else {
            class_wts.at<float>(0) = static_cast<float>(neg_ex) /
                                     static_cast<float>(pos_ex + neg_ex);
            class_wts.at<float>(1) = static_cast<float>(pos_ex) /
                                     static_cast<float>(pos_ex + neg_ex);
        }
        class_wts_cv = class_wts;
        svmParams.class_weights = &class_wts_cv;
    }
}
}
}

#include "tinker_object_recognition/graph_recognition/bow_classification.h" 
#include <ros/ros.h>
#include <cstdio>

using namespace tinker::vision;
using std::string;
using std::vector;

class BoWTrainNode {
public:
    BoWTrainNode() 
        :private_nh_("~") {
        XmlRpc::XmlRpcValue image_class_info;
        private_nh_.getParam("image_class_info", image_class_info);
        ROS_ASSERT(image_class_info.size() > 0);
        private_nh_.getParam("vocabulary_file_name", vocabulary_filename_);
        ROS_ASSERT(vocabulary_filename_.size() > 0);
        bow_recognition_ = BoWRecognition(image_class_info);
    }

    void Train() {
        cv::Mat vocabulary = bow_recognition_.TrainVocabulary();
        bow_recognition_.WriteVocabulary(vocabulary_filename_, vocabulary);
        CvSVM svm;
        vector<string> object_classes = bow_recognition_.GetObjectClasses();
        for(int i = 0; i < object_classes.size(); i++) {
            bow_recognition_.TrainSVMClassifier(svm, object_classes[i]);
            bow_recognition_.WriteSVMClassifer(svm, object_classes[i]);
        }
    }
private:
    BoWRecognition bow_recognition_;
    string vocabulary_filename_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
};

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "arm_bow_train");
    ROS_INFO("Bag of words training...");
    BoWTrainNode train_node;
    train_node.Train();
    return 0;
}

#include "tinker_object_recognition/graph_recognition/bow_classification.h"
#include "tinker_object_recognition/arm_target_finder/ForegroundDetector.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <tinker_object_recognition/FindObjects.h>

using namespace tinker::vision;
using std::string;
using std::vector;

struct ClassifyResult {
    bool found;
    string name;
    double div_x;
    double div_y;
};

class BoWClassifyNode {
public:
    BoWClassifyNode()
        : private_nh_("~"),
          detector_(8, 0.265),
          svms(NULL),
          debug_seq_(0),
          seq_(0),
          new_frame_(false) {
        XmlRpc::XmlRpcValue image_class_info;
        private_nh_.param("image_class_info", image_class_info);
        private_nh_.param("vocabulary_file_name", vocabulary_filename_);
        bow_recognition_ = BoWRecognition(image_class_info);
        bow_recognition_.ReadVocabulary(vocabulary_filename_);
        object_classes = bow_recognition_.GetObjectClasses();
        svms = new CvSVM[object_classes.size()];
        ros::NodeHandle nh;
        find_object_server_ = nh.advertiseService(
            "arm_find_objects", &BoWClassifyNode::FindObjectService, this);
        for (int i = 0; i < object_classes.size(); i++) {
            bow_recognition_.ReadSVMClassifier(svms[i], object_classes[i]);
        }
    }

    void ImageCallBack(const sensor_msgs::Image::ConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cam_mat = cv_ptr->image;
        result_ = Classify(cam_mat);
        objects_ = object_recognition_msgs::RecognizedObjectArray();
        std_msgs::Header header;
        header.seq = seq_++;
        header.stamp = ros::Time::now();
        header.frame_id = "hand";
        objects_.header = header;
        if (result_.found) {
            object_recognition_msgs::RecognizedObject object;
            object.header = header;
            object.type.key = result_.name;
            object.pose.pose.pose.position.x = result_.div_x;
            object.pose.pose.pose.position.y = result_.div_y;
            objects_.objects.push_back(object);
        }
        new_frame_ = true;
        pub_.publish(objects_);
    }

    ClassifyResult Classify(const cv::Mat &image) {
        ClassifyResult result;
        result.found = false;
        cv::Mat res_mat;
        cv::Rect bound;
        try {
            int find = detector_.CutForegroundOut(image, res_mat, bound);
            if (find == tinker::vision::ForegroundDetector::DETECTED) {
                // publish it
                sensor_msgs::Image pubmsg;
                cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", res_mat);
                cvi.toImageMsg(pubmsg);
                cvi.header.seq = debug_seq_++;
                cvi.header.frame_id = "hand";
                cvi.header.stamp = ros::Time::now();
                debug_pub_.publish(pubmsg);

                cv::Mat bow_descriptor;
                bow_recognition_.CalculateImageDescriptor(image,
                                                          bow_descriptor);

                int positive_count = 0;

                for (int i = 0; i < object_classes.size(); i++) {
                    if (svms[i].predict(bow_descriptor) > 0) {
                        result.name = object_classes[i];
                        positive_count++;
                    }
                }
                result.found = (positive_count == 1);
                if (result.found) {
                    double center_x = bound.x + bound.width / 2.;
                    double center_y = bound.y + bound.height / 2.;
                    result.div_x = center_x - image.cols / 2.;
                    result.div_y = center_y - image.rows / 2.;
                }
            }
        } catch (cv::Exception &e) {
            ROS_WARN("failed to recognize: %s", e.what());
        }
        return result;
    }

    bool FindObjectService(
        tinker_object_recognition::FindObjects::Request &req,
        tinker_object_recognition::FindObjects::Response &res) {
        new_frame_ = false;
        while (!new_frame_) ros::spinOnce();
        res.success = result_.found;
        res.objects = objects_;
        return true;
    }

    ~BoWClassifyNode() {
        if (svms) delete [] svms;
    }
private:
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    BoWRecognition bow_recognition_;
    ForegroundDetector detector_;
    string vocabulary_filename_;
    ros::Publisher debug_pub_;
    ros::Publisher pub_;
    ros::ServiceServer find_object_server_;
    vector<string> object_classes;
    CvSVM * svms;
    ClassifyResult result_;
    object_recognition_msgs::RecognizedObjectArray objects_;
    int debug_seq_;
    int seq_;
    bool new_frame_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "arm_bow_classify");
    BoWClassifyNode n;
    ros::spin();
    return 0;
}

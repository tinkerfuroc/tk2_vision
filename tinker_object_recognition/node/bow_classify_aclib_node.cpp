#include "tinker_object_recognition/graph_recognition/bow_classification.h"
#include "tinker_object_recognition/arm_target_finder/ForegroundDetector.h"
#include "tinker_object_recognition/utilities.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <boost/thread/mutex.hpp>
#include <actionlib/server/simple_action_server.h>
#include <tinker_vision_msgs/ObjectAction.h>
#include <tinker_vision_msgs/ObjectClassify.h>
#include <boost/foreach.hpp>
#include <cfloat>
#include <cmath>
#include <map>

using namespace tinker::vision;
using std::string;
using std::vector;
using std::map;

struct ClassifyResult {
    bool found;
    int id;
    string name;
    float div_x;
    float div_y;
    double df_val;
};

class BoWClassifyServerNode {
public:
    BoWClassifyServerNode()
        : private_nh_("~"),
          detector_(),
          svms(NULL),
          debug_seq_(0),
          seq_(0),
          save_seq_(0),
          img_count_(0),
          as_(nh_, "arm_find_objects", false) {
        detector_.setParam(private_nh_);
        XmlRpc::XmlRpcValue image_class_info;
        private_nh_.getParam("image_class_info", image_class_info);
        private_nh_.getParam("vocabulary_file_name", vocabulary_filename_);
        bow_recognition_ = BoWRecognition(image_class_info);
        bow_recognition_.ReadVocabulary(vocabulary_filename_);
        object_classes = bow_recognition_.GetObjectClasses();
        svms = new CvSVM[object_classes.size()];
        sub_ = nh_.subscribe("tk2_com/arm_cam_image", 1,
                             &BoWClassifyServerNode::ImageCallBack, this);
        pub_ = nh_.advertise<object_recognition_msgs::RecognizedObjectArray>(
            "arm_cam_objects", 1);
        dbg_pub_ =
            nh_.advertise<sensor_msgs::Image>("tk2_vision/dbg_handimg", 1);
        for (int i = 0; i < object_classes.size(); i++) {
            bow_recognition_.ReadSVMClassifier(svms[i], object_classes[i]);
        }
        as_.registerGoalCallback(
            boost::bind(&BoWClassifyServerNode::goalCB, this));
        as_.registerPreemptCallback(
            boost::bind(&BoWClassifyServerNode::preemptCB, this));
        try {
            as_.start();
        } catch (...) {
            ROS_ERROR("server start failed =_=");
        }
        classify_service_ = nh_.advertiseService(
            "object_classify", &BoWClassifyServerNode::ClassifyService, this);
    }

    void goalCB() {
        found_count_ = vector<int>(object_classes.size(), 0);
        found_div_x_ = vector<float>(object_classes.size(), 0);
        object_results_ =
            vector<object_recognition_msgs::RecognizedObjectArray>(
                object_classes.size());
        tinker_vision_msgs::ObjectGoalConstPtr goal = as_.acceptNewGoal();
        sample_count_ = goal->sample_count;
        accept_count_ = goal->accept_count;
        if (sample_count_ == 0 && accept_count_ == 0) {
            sample_count_ = 10;
            accept_count_ = 5;
        }
        count_ = sample_count_;
        seq_++;
    }

    void preemptCB() { as_.setPreempted(); }

    void ImageCallBack(const sensor_msgs::Image::ConstPtr &msg) {
        if (!as_.isActive()) {
            // ROS_ERROR("action server is not active =.=");
            return;
        }
        img_count_++;
        if (img_count_ % 3 != 0) return;
        ROS_INFO("Classifying");
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cam_mat = cv_ptr->image;
        vector<ClassifyResult> classify_results = DetectObject(cam_mat);
        std_msgs::Header header;
        header.seq = seq_;
        header.stamp = ros::Time::now();
        header.frame_id = "hand";
        BOOST_FOREACH(ClassifyResult &classify_result, classify_results) {
            ROS_INFO("Found %s at %f", classify_result.name.c_str(), classify_result.div_x);
            object_recognition_msgs::RecognizedObject object;
            object_recognition_msgs::RecognizedObjectArray objects = object_recognition_msgs::RecognizedObjectArray();
            objects.header = header;
            object.header = header;
            object.type.key = classify_result.name;
            object.pose.pose.pose.position.x = classify_result.div_x;
            object.pose.pose.pose.position.y = classify_result.div_y;
            objects.objects.push_back(object);
            found_count_[classify_result.id]++;
            object_results_[classify_result.id] = objects;
            found_div_x_[classify_result.id] = fabs(classify_result.div_x);
        }
        count_--;
        if (count_ == 0) {
            int recognized_class_id = 0;
            float min_div_x_ = FLT_MAX;
            bool found = false;
            for (int i = 0; i < object_classes.size(); i++) {
                ROS_INFO("%s: %d", 
                        object_classes[i].c_str(),
                        found_count_[i]);
                if (found_count_[i] > accept_count_ && found_div_x_[i] < min_div_x_) {
                    recognized_class_id = i;
                    found = true;
                    min_div_x_ = found_div_x_[i];
                }
            }
            if (found) {
                ROS_INFO("Finally found %s at div x %f",
                        object_classes[recognized_class_id].c_str(),
                        found_div_x_[recognized_class_id]);
                act_result_.success = true;
                act_result_.objects = object_results_[recognized_class_id];
                // set the action state to succeeded
                as_.setSucceeded(act_result_);
            } else {
                act_result_.success = false;
                act_result_.objects =
                    object_recognition_msgs::RecognizedObjectArray();
                // set the action state to aborted
                as_.setAborted(act_result_);
            }
        }
    }

    vector<ClassifyResult> DetectObject(const cv::Mat &image) {
        vector<ClassifyResult> results;
        cv::Mat res_mat;
        cv::Rect bound;
        vector<ForegroundImage> foregrounds = detector_.CutAllForegroundOut(image);
        BOOST_FOREACH(ForegroundImage &foreground, foregrounds) {
            DebugPubImage(foreground.foreground);
            ClassifyResult result;
            if (Classify(foreground.foreground, result)) {
                cv::Rect &bound = foreground.bound;
                float center_x = bound.x + bound.width / 2.;
                float bottom_y = bound.y + bound.height;
                result.div_x = center_x - image.cols / 2.;
                result.div_y = bottom_y - image.rows / 2.;
                results.push_back(result);
            }
        }
        return results;
    }

    bool Classify(const cv::Mat image, ClassifyResult &result) {
        cv::Mat bow_descriptor;
        result.found = false;
        // res_mat = HistogramEqualizeRGB(res_mat);
        try {
            bow_recognition_.CalculateImageDescriptor(image,
                                                      bow_descriptor);
            int positive_count = 0;

            for (int i = 0; i < object_classes.size(); i++) {
                if (svms[i].predict(bow_descriptor) > 0) {
                    double df_val = svms[i].predict(bow_descriptor, true);
                    ROS_INFO("found df_val %lf for class %s", df_val,
                              object_classes[i].c_str());
                    result.name = object_classes[i];
                    result.id = i;
                    result.df_val = df_val;
                    positive_count++;
                }
            }
            if (positive_count < 1) ROS_DEBUG("No found pair");
            if (positive_count > 1) ROS_DEBUG("Too much found pair");
            result.found = (positive_count == 1);
            return result.found;
        } catch (cv::Exception &e) {
            ROS_DEBUG("failed to build bow descriptor: %s", e.what());
            return false;
        }
    } 

    void DebugPubImage(const cv::Mat &image) {
        sensor_msgs::Image img;
        cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", image);
        cvi.header.seq = debug_seq_++;
        cvi.header.frame_id = "hand";
        cvi.header.stamp = ros::Time::now();
        cvi.toImageMsg(img);
        act_feedback_.handimg = img;
        dbg_pub_.publish(img);
        as_.publishFeedback(act_feedback_);
    }

    bool ClassifyService(tinker_vision_msgs::ObjectClassify::Request &req,
                         tinker_vision_msgs::ObjectClassify::Response &res) {
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(req.img, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        ClassifyResult result;
        Classify(img, result);
        res.found = result.found;
        res.name.data = result.name;
        res.df_val = result.df_val;
        return true;
    }

    ~BoWClassifyServerNode() {
        if (svms) delete[] svms;
    }

private:
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    BoWRecognition bow_recognition_;
    ForegroundDetector detector_;
    string vocabulary_filename_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    vector<string> object_classes;
    CvSVM *svms;
    int debug_seq_;
    int seq_;
    int save_seq_;
    int count_;
    int sample_count_;
    int accept_count_;
    int img_count_;

    double entropy_threshold_;
    int filter_size_;
    float max_aspect_ratio_tolerance_;

    vector<int> found_count_;
    vector<float> found_div_x_;
    vector<object_recognition_msgs::RecognizedObjectArray> object_results_;

    actionlib::SimpleActionServer<tinker_vision_msgs::ObjectAction> as_;
    tinker_vision_msgs::ObjectFeedback act_feedback_;
    tinker_vision_msgs::ObjectResult act_result_;

    ros::ServiceServer classify_service_;

    ros::Publisher dbg_pub_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "arm_bow_classify_server");
    BoWClassifyServerNode n;
    ros::spin();
    return 0;
}

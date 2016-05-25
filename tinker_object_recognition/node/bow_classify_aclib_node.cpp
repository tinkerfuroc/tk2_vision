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

using namespace tinker::vision;
using std::string;
using std::vector;

struct ClassifyResult {
    bool found;
    int id;
    string name;
    double div_x;
    double div_y;
};

class BoWClassifyServerNode {
public:
    BoWClassifyServerNode()
        : private_nh_("~"),
          detector_(),
          svms(NULL),
          debug_seq_(0),
          seq_(0),
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
        dbg_pub_ = nh_.advertise<sensor_msgs::Image>("tk2_vision/dbg_handimg", 1);
        for (int i = 0; i < object_classes.size(); i++) {
            bow_recognition_.ReadSVMClassifier(svms[i], object_classes[i]);
        }
        as_.registerGoalCallback(boost::bind(&BoWClassifyServerNode::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&BoWClassifyServerNode::preemptCB, this));
        try
        {
            as_.start();
        }
        catch(...)
        {
            ROS_ERROR("server start failed =_=");
        }
    }

    void goalCB()
    {
        found_count_ = vector<int>(object_classes.size(), 0);
        object_results_ = vector<object_recognition_msgs::RecognizedObjectArray>(object_classes.size());
        tinker_vision_msgs::ObjectGoalConstPtr goal = as_.acceptNewGoal(); 
        sample_count_ = goal->sample_count;
        accept_count_ = goal->accept_count;
        if (sample_count_ == 0 && accept_count_ == 0) {
            sample_count_ = 10;
            accept_count_ = 5;
        }
        count_ = sample_count_;
    }
    
    void preemptCB()
    {
        as_.setPreempted();
    } 

    void ImageCallBack(const sensor_msgs::Image::ConstPtr &msg) {
        if (!as_.isActive()) 
        {
            //ROS_ERROR("action server is not active =.=");
            return;
        }
        img_count_++;
        if (img_count_ % 10 != 0) return; 
        ROS_DEBUG("Classifying");
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
        count_--;
        if (result_.found) {
            ROS_INFO("Found %s", result_.name.c_str());
            found_count_[result_.id]++;
            object_results_[result_.id] = objects_;
        } 
        if(count_ == 0)
        {
            int recognized_class_count = 0;
            int recognized_class_id = 0;
            for (int i = 0; i < object_classes.size(); i++) {
                if (found_count_[i] > accept_count_) {
                    recognized_class_id = i;
                    recognized_class_count++;
                }
            }
            if (recognized_class_count == 1) {
                act_result_.success = true;
                act_result_.objects = object_results_[recognized_class_id];
                // set the action state to succeeded
                as_.setSucceeded(act_result_);
            }
            else
            {
                act_result_.success = false;
                act_result_.objects = object_recognition_msgs::RecognizedObjectArray();
                //set the action state to aborted
                as_.setAborted(act_result_);
            }
        }
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
                sensor_msgs::Image img;
                cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", res_mat);
                cvi.header.seq = debug_seq_++;
                cvi.header.frame_id = "hand";
                cvi.header.stamp = ros::Time::now();
                cvi.toImageMsg(img);
                act_feedback_.handimg = img;
                dbg_pub_.publish(img);
                as_.publishFeedback(act_feedback_);

                cv::Mat bow_descriptor;
                res_mat = HistogramEqualizeRGB(res_mat);
                bow_recognition_.CalculateImageDescriptor(res_mat,
                                                          bow_descriptor);

                int positive_count = 0;

                for (int i = 0; i < object_classes.size(); i++) {
                    if (svms[i].predict(bow_descriptor) > 0) {
                        double df_val = svms[i].predict(bow_descriptor, true);
                        ROS_DEBUG("found df_val %lf for class %s", df_val,
                                  object_classes[i].c_str());
                        result.name = object_classes[i];
                        result.id = i;
                        positive_count++;
                    }
                }
                if (positive_count < 1) ROS_DEBUG("No found pair");
                if (positive_count > 1) ROS_DEBUG("Too much found pair");
                result.found = (positive_count == 1);
                if (result.found) {
                    //ROS_INFO("Found %s", result.name.c_str());
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
    ClassifyResult result_;
    object_recognition_msgs::RecognizedObjectArray objects_;
    int debug_seq_;
    int seq_;
    int count_;
    int sample_count_;
    int accept_count_;
    int img_count_;
    
    double entropy_threshold_;
    int filter_size_;
    float max_aspect_ratio_tolerance_;
    
    vector<int> found_count_;
    vector<object_recognition_msgs::RecognizedObjectArray> object_results_;
    
    actionlib::SimpleActionServer<tinker_vision_msgs::ObjectAction> as_;
    tinker_vision_msgs::ObjectFeedback act_feedback_;
    tinker_vision_msgs::ObjectResult act_result_;
    
    ros::Publisher dbg_pub_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "arm_bow_classify_server");
    BoWClassifyServerNode n;
    ros::spin();
    return 0;
}

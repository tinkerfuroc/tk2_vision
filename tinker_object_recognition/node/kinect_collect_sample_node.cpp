#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <cassert>
#include <fstream>
#include "tinker_object_recognition/utilities.h"
#include <boost/bind.hpp>

using namespace tinker::vision;
using namespace std;
using namespace cv;


enum CollectState { WAITING, BOUNDING, COLLECTING };


class Collector {
public:
    Collector()
        : private_nh_("~"),
          finished_(false),
          class_counter_(0),
          no_counter_(0),
          state_(WAITING),
          img_counter_(0) {
        cv::namedWindow("kinect");
        init();
    }

    void OnClick(int event, int x, int y, int flags, void *userdata) {
        if (event != cv::EVENT_LBUTTONDOWN) return;
        int buf;
        switch(state_) {
        case WAITING:
            from_x_ = x;
            from_y_ = y;
            state_ = BOUNDING;
            break;
        case BOUNDING:
            to_x_ = x;
            to_y_ = y;
            if (to_x_ < from_x_) {
                buf = to_x_; to_x_ = from_x_; from_x_ = buf;
            }
            if (to_y_ < from_y_) {
                buf = to_y_; to_y_ = from_y_; from_y_ = buf;
            }
            state_ = COLLECTING;
            break;
        case COLLECTING:
            break;
        }
        ROS_INFO("Click at %d %d\n", x, y);
    }

    void init() {
        // read param
        XmlRpc::XmlRpcValue image_class_info;
        private_nh_.getParam("image_class_info", image_class_info);
        ROS_ASSERT(image_class_info.size() > 0);
        ROS_ASSERT(image_class_info.hasMember("object_classes"));
        ROS_ASSERT(image_class_info.hasMember("image_num"));
        ROS_ASSERT(image_class_info.hasMember("image_folder_name"));
        ROS_ASSERT(image_class_info["image_num"].getType() ==
                   XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(image_class_info["object_classes"].getType() ==
                   XmlRpc::XmlRpcValue::TypeArray);
        filepath_ = std::string(image_class_info["image_folder_name"]);
        ROS_ASSERT(filepath_.size() > 0);
        object_num_ = (int)image_class_info["image_num"];
        XmlRpc::XmlRpcValue object_classes = image_class_info["object_classes"];
        for (int i = 0; i < object_classes.size(); i++) {
            ROS_ASSERT(object_classes[i].getType() ==
                       XmlRpc::XmlRpcValue::TypeString);
            string object_class = (string)object_classes[i];
            objectclasses_.push_back(object_class);
        }
        string topic_name;
        private_nh_.param("image_topic", topic_name, string("/kinect/rgb/image"));
        sub_ = nh_.subscribe(topic_name, 1,
                             &Collector::ArmCamImgCallback, this);
        ROS_INFO("waiting for %s ...", pngNameWrapper().c_str());
    }

    void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr &msg) {
        // get foreground
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cam_mat = cv_ptr->image;
        img_counter_++;
        int key = cvWaitKey(100);
        if (!finished_ && state_ == COLLECTING) {
            cv::Rect bound;
            bound.x = from_x_;
            bound.y = from_y_;
            bound.width = to_x_ - from_x_;
            bound.height = to_y_ - from_y_;
            string filename = pngNameWrapper();
            string filepath = filepath_ + "/" + filename;
            if (img_counter_ % 5 == 0) {
                imwrite(filepath, cam_mat(bound));
                ROS_INFO("%s saved", filename.c_str());
            }
            cv::rectangle(cam_mat, cv::Point(from_x_, from_y_), 
                    cv::Point(to_x_, to_y_), cv::Scalar(0, 255, 0));
            // next object
            no_counter_++;
            if (no_counter_ == object_num_) {
                no_counter_ = 0;
                class_counter_++;
                if (class_counter_ == objectclasses_.size()) {
                    finished_ = true;
                    ROS_INFO("Done.");
                    return;
                }
                state_ = WAITING;
            }
            filename = pngNameWrapper();
            ROS_INFO("waiting for %s ...", filename.c_str());
        } 
        cv::imshow("kinect", cam_mat);
        return;
    }

    string pngNameWrapper() {
        static char buffer[66];
        sprintf(buffer, "%s_%d.png", objectclasses_[class_counter_].c_str(),
                no_counter_);
        return string(buffer);
    }

private:
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    string filepath_;

    CollectState state_;
    int no_counter_;
    int class_counter_;
    int img_counter_;
    int object_num_;
    int from_x_;
    int from_y_;
    int to_x_;
    int to_y_;
    bool finished_;
    bool waiting_area_;
    vector<string> objectclasses_;
};

void OnMouse(int event, int x, int y, int flags, void *userdata) {
    Collector *c = (Collector *)userdata;
    c->OnClick(event, x, y, flags, NULL);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "collect_sample_node");
    ROS_INFO("press key 's' to save picture.");
    Collector c;
    cv::setMouseCallback("kinect", OnMouse, &c);
    ros::spin();
    return 0;
}

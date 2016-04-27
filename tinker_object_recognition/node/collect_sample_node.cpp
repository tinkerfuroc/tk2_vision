//
// Du 2016.4.9.
//

#include <tinker_object_recognition/arm_target_finder/ForegroundDetector.h>
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
using namespace tinker::vision;
using namespace std;
using namespace cv;


class Collector
{
public:
    Collector() 
        :private_nh_("~"), fd_(8, 0.265), listfilename("list.txt"), 
         finished_(false), class_counter_(0), no_counter_(0) {
        init();
    }
    
    void init() {
        private_nh_.getParam("filepath", filepath_);
        ROS_ASSERT(filepath_.size() > 0);
        string listfilepath = filepath_ + "/" + listfilename;
        listfile_.open(listfilepath.c_str());
        //read param
        XmlRpc::XmlRpcValue image_class_info;
        private_nh_.getParam("image_class_info", image_class_info);
        ROS_ASSERT(image_class_info.size() > 0);
        ROS_ASSERT(image_class_info.hasMember("object_classes"));
        ROS_ASSERT(image_class_info.hasMember("image_num"));
        ROS_ASSERT(image_class_info.hasMember("image_folder_name"));
        ROS_ASSERT(image_class_info["image_num"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(image_class_info["object_classes"].getType() == XmlRpc::XmlRpcValue::TypeArray);
        string image_dir = image_class_info["image_folder_name"];
        object_num_ = (int)image_class_info["image_num"];
        XmlRpc::XmlRpcValue object_classes = image_class_info["object_classes"];
        for (int i = 0 ; i < object_classes.size(); i++) {
            ROS_ASSERT(object_classes[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            string object_class = (string)object_classes[i];
            objectclasses_.push_back(object_class);
        }
        sub_ = nh_.subscribe("tk2_com/arm_cam_image", 1, 
            &Collector::ArmCamImgCallback, this);
        ROS_INFO("waiting for %s ...", pngNameWrapper().c_str());
    }
    
    void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr &msg) {
        // get foreground
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cam_mat = cv_ptr->image;
        cv::Mat res_mat;
        if(fd_.CutForegroundOut(cam_mat, res_mat) == ForegroundDetector::DETECTED)
        {
            if (res_mat.cols > 0 && res_mat.rows > 0) 
            {
                cv::imshow("view", res_mat);
                int key = cvWaitKey(100);
                if (key == 1048691 || key == 1179731 || key == 115)  //'s'||'S'
                {
                    if (!finished_) {
                        string filename = pngNameWrapper();
                        string filepath = filepath_ + "/" + filename;
                        // save as png, compression level at 9.
                        vector<int> compression_params;
                        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                        compression_params.push_back(9);
                        imwrite(filepath, res_mat, compression_params);
                        listfile_ << filename << endl;
                        ROS_INFO("%s saved", filename.c_str());
                        //next object
                        no_counter_++;
                        if (no_counter_ == object_num_) {
                            no_counter_ = 0;
                            class_counter_++;
                            if (class_counter_ == objectclasses_.size()) {
                                finished_ = true;
                                ROS_INFO("Done.");
                                return;
                            }
                        }
                        filename = pngNameWrapper();
                        ROS_INFO("waiting for %s ...", filename.c_str());
                    } 
                }
                return;
            }
        }
        return;
    }
    
    string pngNameWrapper() {
        static char buffer[66];
        sprintf(buffer, "%s_%d.png", objectclasses_[class_counter_].c_str(), no_counter_);
        return string(buffer);
    }
    
   

private:
    ForegroundDetector fd_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    
    ofstream listfile_;
    string filepath_;
    string listfilename;
    
    int no_counter_;
    int class_counter_;
    int object_num_;
    bool finished_;
    vector<string> objectclasses_;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "collect_sample_node");
    ROS_INFO("press key 's' to save picture.");
    Collector c;
    
    ros::spin();
    return 0;
}

//
// Du 2016.1.15.
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
#include <tinker_object_recognition/tinyxml2.h>
#include "tinker_object_recognition/utilities.h"
using namespace tinyxml2;
using namespace tinker::vision;
using namespace std;
using namespace cv;

class NameGenerator {
public:
    NameGenerator(int object_num, std::vector<std::string> subject_name)
        : object_num_(object_num), subject_name_(subject_name) {
        init();
    }

    void init() {
        object_counter_ = subject_counter_ = 0;
        finished = false;
    }

    char *GetNextName() {
        if (!finished) {
            sprintf(buffer, "%s_%d.png",
                    subject_name_[subject_counter_].c_str(), object_counter_);
            object_counter_++;
            if (object_counter_ == object_num_) {
                object_counter_ = 0;
                subject_counter_++;
                if (subject_counter_ == subject_name_.size()) {
                    finished = true;
                }
            }
        } else {
            sprintf(buffer, "nothing");
        }
        return buffer;
    }

    bool finished;

private:
    char buffer[66];
    int object_counter_;
    int subject_counter_;
    int object_num_;
    std::vector<std::string> subject_name_;
} * ng;

void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr &msg);
int Usage();

tinker::vision::ForegroundDetector fd(8, 0.265);

int object_num;
char *filename;
const string listfilename = "list.txt";
std::ofstream listfile;
bool listfileclosed = false;

void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr &msg) {
    // get foreground
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat cam_mat = cv_ptr->image;
    cv::Mat res_mat;
    if(fd.CutForegroundOut(cam_mat, res_mat) == ForegroundDetector::DETECTED)
    {
        //res_mat = HistogramEqualizeRGB(res_mat);
        if (res_mat.cols > 0 && res_mat.rows > 0) {
            ROS_INFO("Found");
            cv::imshow("view", res_mat);
            int key = cvWaitKey(100);
            if (key == 1048691 || key == 1179731 || key == 115)  //'s'||'S'
            {
                if (!ng->finished) {
                    // save as png, compression level at 9.
                    std::vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                    compression_params.push_back(9);
                    imwrite(filename, res_mat, compression_params);
                    listfile << filename << std::endl;
                    ROS_INFO("%s saved", filename);
                    filename = ng->GetNextName();
                    ROS_INFO("waiting for %s ...", filename);
                } else {
                    if (!listfileclosed) {
                        listfile.close();
                        listfileclosed = true;
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv) {
    if (argc != 2) return Usage();
    std::string xmlfilename(argv[1]);

    ros::init(argc, argv, "arm_webcam_sample_collector");
    ros::NodeHandle nh;
    ROS_INFO("press key 's' to save picture.");

    listfile.open(listfilename.c_str());

    object_num = 0;
    std::vector<std::string> subject_name;
    ROS_INFO("Loading xml");
    try {
        XMLDocument doc;
        doc.LoadFile(xmlfilename.c_str());
        XMLElement *sample = doc.FirstChildElement("sample");
        object_num = atoi(sample->FirstAttribute()->Value());
        for (XMLElement *object = sample->FirstChildElement("subject"); object;
             object = object->NextSiblingElement()) {
            const XMLAttribute *attributeOfobject = object->FirstAttribute();
            ROS_INFO("Class %s", attributeOfobject->Value());
            subject_name.push_back(attributeOfobject->Value());
        }

    } catch (...) {
        ROS_INFO("error occured when loading xml file.");
        return -1;
    }
    ng = new NameGenerator(object_num, subject_name);

    filename = ng->GetNextName();
    ROS_INFO("waiting for %s ...", filename);

    ros::Subscriber sub =
        nh.subscribe("tk2_com/arm_cam_image", 1, ArmCamImgCallback);
    ros::spin();
    delete ng;
    return 0;
}

int Usage() {
    ROS_INFO("usage: collectsample ***.xml");
    return 0;
}

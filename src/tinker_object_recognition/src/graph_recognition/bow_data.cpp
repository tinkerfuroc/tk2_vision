#include "tinker_object_recognition/graph_recognition/bow_data.h"
#include <ros/ros.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>

using std::string;
using std::vector;
using std::cout;
using std::endl;

namespace tinker {
namespace vision {

void BoWData::InitBoWData(XmlRpc::XmlRpcValue & data_info) {
    int object_num = 0;
    ROS_ASSERT(data_info.hasMember("object_classes"));
    ROS_ASSERT(data_info.hasMember("image_num"));
    ROS_ASSERT(data_info.hasMember("image_folder_name"));
    ROS_ASSERT(data_info["image_num"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(data_info["object_classes"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    string image_dir = data_info["image_folder_name"];
    object_num = (int)data_info["image_num"];
    XmlRpc::XmlRpcValue object_classes = data_info["object_classes"];
    ROS_INFO("Object classes:");
    for (int i = 0 ; i < object_classes.size(); i++) {
        ROS_ASSERT(object_classes[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        string object_class = (string)object_classes[i];
        ROS_INFO("%s", object_class.c_str());
        objectclasses_.push_back(object_class);
    }
    int counter = 0;
    int subject_counter_ = 0;
    int object_counter_ = 0;
    static char buffer[66];
    while (1) {
        sprintf(buffer, "%s_%d.png", objectclasses_[subject_counter_].c_str(),
                object_counter_);
        string imgpath = image_dir + "/" + string(buffer);
        ObdImage o(IntegerToString(counter), imgpath,
                   objectclasses_[subject_counter_]);
        images_.push_back(o);
        object_counter_++;
        counter++;
        if (object_counter_ == object_num) {
            object_counter_ = 0;
            subject_counter_++;
            if (subject_counter_ == objectclasses_.size()) {
                break;
            }
        }
    }
}

void BoWData::GetClassImages(const string& obj_class,
                             const ObdDatasetType dataset,
                             vector<ObdImage>& images,
                             vector<char>& object_present) {
    for (int i = 0; i < images_.size(); i++) {
        ObdImage o = images_[i];
        images.push_back(o);
        object_present.push_back(o.classname == obj_class);
    }
}

}
}

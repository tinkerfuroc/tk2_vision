#ifndef __TINKER_POINTCLOUD_OBJECT_FINDER_H__
#define __TINKER_POINTCLOUD_OBJECT_FINDER_H__

#include <string>
#include <vector>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>
#include "tinker_object_recognition/common.h"

namespace tinker {
namespace vision {

class PointCloudObjectFinder {
public:
    PointCloudObjectFinder();
    void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & camera_info);
    void DepthCallback(const sensor_msgs::Image::ConstPtr & depth_image);
    void RGBCallback(const sensor_msgs::Image::ConstPtr & rgb_image);
    bool StartService(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
    bool PendService(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
    void TimerCallback(const ros::TimerEvent & event);
private:
    std::vector<PointCloudPtr> GetObjectPointClouds();
    cv::Mat depth_image_;
    cv::Mat rgb_image_;
    std::string depth_topic_name_;
    std::string rgb_topic_name_;
    ros::Subscriber depth_image_subscribe_;
    ros::Subscriber rgb_image_subscribe_;
    ros::Publisher object_pub_;
    ros::Publisher debug_pub_;
    bool depth_ready_;
    bool rgb_ready_;
    bool running_;
    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    std::string frame_id_;
    int object_seq_;
    int debug_seq_;
};

}
}

#endif


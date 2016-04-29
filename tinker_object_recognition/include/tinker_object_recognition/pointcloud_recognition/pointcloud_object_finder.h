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
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <tinker_vision_msgs/FindObjects.h>
#include <boost/thread/mutex.hpp>

namespace tinker {
namespace vision {

class PointCloudObjectFinder {
public:
    PointCloudObjectFinder();
    void CameraInfoCallback(
        const sensor_msgs::CameraInfo::ConstPtr& camera_info);
    void DepthCallback(const sensor_msgs::Image::ConstPtr& depth_image);
    void RGBCallback(const sensor_msgs::Image::ConstPtr& rgb_image);
    bool StartService(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res);
    bool PendService(std_srvs::Trigger::Request& req,
                     std_srvs::Trigger::Response& res);
    bool FindObjectService(
        tinker_vision_msgs::FindObjects::Request& req,
        tinker_vision_msgs::FindObjects::Response& res);
    void TimerCallback(const ros::TimerEvent& event);

private:
    object_recognition_msgs::RecognizedObjectArray GetRecognizedObjects();
    std::vector<PointCloudPtr> GetObjectPointClouds();
    void StartSubscribe();
    void StopSubscribe();
    cv::Mat depth_image_;
    cv::Mat rgb_image_;
    std::string depth_topic_name_;
    std::string rgb_topic_name_;
    ros::Subscriber depth_image_subscribe_;
    ros::Subscriber rgb_image_subscribe_;
    ros::Publisher object_pub_;
    ros::Publisher debug_pub_;
    ros::ServiceServer start_server_;
    ros::ServiceServer pend_server_;
    ros::ServiceServer find_object_server_;
    bool depth_ready_;
    bool rgb_ready_;
    bool topic_running_;
    bool service_running_;
    bool running_;
    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    std::string frame_id_;
    int object_seq_;
    int debug_seq_;
    boost::mutex mutex_;
};
}
}

#endif

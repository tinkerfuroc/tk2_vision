#include "tinker_object_recognition/pointcloud_recognition/pointcloud_object_finder.h"
#include "tinker_object_recognition/pointcloud_recognition/ClusterDivider.h"
#include "tinker_object_recognition/pointcloud_rebuild/pointcloud_builder.h"
#include "tinker_object_recognition/graph_filter/ColorFixer.h"
#include "tinker_object_recognition/graph_filter/filters.h"
#include "tinker_object_recognition/utilities.h"
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

namespace tinker {
namespace vision {

using std::vector;
using std::string;

PointCloudObjectFinder::PointCloudObjectFinder()
    :depth_ready_(false), rgb_ready_(false),
    running_(true), private_nh("~/"), nh("/"),
    object_seq_(0), debug_seq_(0){
    object_pub_ = nh.advertise<object_recognition_msgs::RecognizedObjectArray>("point_cloud_objects", 1);
    //debug_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("debug_output", 1);
    private_nh.param("depth_topic_name", depth_topic_name_, string("/depth/image"));
    private_nh.param("rgb_topic_name", rgb_topic_name_, string("/rgb/image"));
    depth_image_subscribe_ = nh.subscribe(depth_topic_name_,
            10, &PointCloudObjectFinder::DepthCallback, this);
    rgb_image_subscribe_ = nh.subscribe(rgb_topic_name_,
            10, &PointCloudObjectFinder::RGBCallback, this);
}

bool PointCloudObjectFinder::PointCloudObjectFinder::StartService(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res) {
    depth_image_subscribe_ = nh.subscribe(depth_topic_name_,
            10, &PointCloudObjectFinder::DepthCallback, this);
    rgb_image_subscribe_ = nh.subscribe(rgb_topic_name_,
            10, &PointCloudObjectFinder::RGBCallback, this);
    depth_ready_ = false;
    rgb_ready_ = false;
    running_ = true;
    res.success = true;
    return true;
}

bool PointCloudObjectFinder::PendService(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res) {
    depth_image_subscribe_.shutdown();
    rgb_image_subscribe_.shutdown();
    depth_ready_ = false;
    rgb_ready_ = false;
    running_ = false;
    res.success = true;
    return true;
}


void PointCloudObjectFinder::DepthCallback(const sensor_msgs::Image::ConstPtr & depth_image) {
    frame_id_ = depth_image->header.frame_id;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_image);
    }
    catch(cv_bridge::Exception & e) {
        ROS_ERROR("depth image convert exception: %s", e.what());
    }
    depth_image_ = cv_ptr->image;
    depth_ready_ = true;
}

void PointCloudObjectFinder::RGBCallback(const sensor_msgs::Image::ConstPtr &  rgb_image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb_image);
    }
    catch(cv_bridge::Exception & e) {
        ROS_ERROR("depth image convert exception: %s", e.what());
    }
    rgb_image_ = cv_ptr->image;
    rgb_ready_ = true;
}

void PointCloudObjectFinder::TimerCallback(const ros::TimerEvent & event) {
    if(running_ && rgb_ready_ && depth_ready_) {
        rgb_ready_ = false;
        depth_ready_ = false;
        vector<PointCloudPtr> object_clouds = GetObjectPointClouds();
        object_recognition_msgs::RecognizedObjectArray recognized_objects;
        std_msgs::Header header;
        header.seq = object_seq_++;
        header.stamp = ros::Time::now();
        header.frame_id = "map";
        recognized_objects.header = header;
        BOOST_FOREACH(PointCloudPtr object_cloud, object_clouds) {
            object_recognition_msgs::RecognizedObject object;
            geometry_msgs::Point center = GetCenter(object_cloud);
            //ROS_INFO("center at %f %f %f", center.x, center.y, center.z);
            sensor_msgs::PointCloud2 cloud = ToROSCloud(object_cloud);
            object.header = header;
            object.pose.header = header;
            object.point_clouds.push_back(cloud);
            object.pose.header = header;
            object.pose.pose.pose.position = center;
            recognized_objects.objects.push_back(object);
        }
        object_pub_.publish(recognized_objects);
    }
}

vector<PointCloudPtr> PointCloudObjectFinder::GetObjectPointClouds() {
    cv::pyrDown(depth_image_, depth_image_);
    cv::pyrDown(rgb_image_, rgb_image_);
    cv::Mat gray_image;
    cv::cvtColor(rgb_image_, gray_image, CV_BGR2GRAY);
    cv::Mat line_mask = LineFilterMask(gray_image, 3.5);
    ApplyMask(line_mask, depth_image_, cv::Vec3s(0, 0, 0));
    ApplyMask(line_mask, rgb_image_, cv::Vec3b(0, 0, 0));
    cv::Mat entropy_mask = EntropyFilterMask(gray_image, 0.4, 5, 2);
    DilateImage(entropy_mask, 3);
    ErodeImage(entropy_mask, 5);
    DilateImage(entropy_mask, 5);
    ApplyMask(entropy_mask, depth_image_, cv::Vec3s(0, 0, 0));
    ApplyMask(entropy_mask, rgb_image_, cv::Vec3b(0, 0, 0));
    PointCloudPtr filtered_cloud = BuildPointCloud(depth_image_, rgb_image_); 
    ClusterDivider divider(filtered_cloud);
    return divider.GetDividedPointClouds();
}

}
}

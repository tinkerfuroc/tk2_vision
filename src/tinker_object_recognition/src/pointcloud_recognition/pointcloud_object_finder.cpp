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
    : depth_ready_(false),
      rgb_ready_(false),
      topic_running_(false),
      service_running_(false),
      running_(false),
      private_nh("~/"),
      nh("/"),
      object_seq_(0),
      debug_seq_(0) {
    object_pub_ = nh.advertise<object_recognition_msgs::RecognizedObjectArray>(
        "point_cloud_objects", 1);
    debug_pub_ =
        private_nh.advertise<sensor_msgs::PointCloud2>("debug_output", 1);
    private_nh.param("depth_topic_name", depth_topic_name_,
                     string("/depth/image"));
    private_nh.param("rgb_topic_name", rgb_topic_name_, string("/rgb/image"));
    start_server_ = private_nh.advertiseService(
        "start", &PointCloudObjectFinder::StartService, this);
    pend_server_ = private_nh.advertiseService(
        "pend", &PointCloudObjectFinder::PendService, this);
    find_object_server_ =
        nh.advertiseService("kinect_find_objects",
                            &PointCloudObjectFinder::FindObjectService, this);
}

bool PointCloudObjectFinder::PointCloudObjectFinder::StartService(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    StartSubscribe();
    topic_running_ = true;
    res.success = true;
    return true;
}

bool PointCloudObjectFinder::PendService(std_srvs::Trigger::Request& req,
                                         std_srvs::Trigger::Response& res) {
    topic_running_ = false;
    res.success = true;
    return true;
}

void PointCloudObjectFinder::DepthCallback(
    const sensor_msgs::Image::ConstPtr& depth_image) {
    frame_id_ = depth_image->header.frame_id;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("depth image convert exception: %s", e.what());
    }
    depth_image_ = cv_ptr->image;
    depth_ready_ = true;
}

void PointCloudObjectFinder::RGBCallback(
    const sensor_msgs::Image::ConstPtr& rgb_image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb_image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("depth image convert exception: %s", e.what());
    }
    rgb_image_ = cv_ptr->image;
    rgb_ready_ = true;
}

void PointCloudObjectFinder::TimerCallback(const ros::TimerEvent& event) {
    if (topic_running_ && rgb_ready_ && depth_ready_) {
        object_recognition_msgs::RecognizedObjectArray recognized_objects =
            GetRecognizedObjects();
        object_pub_.publish(recognized_objects);
    }
    if (!topic_running_ && !service_running_) {
        StopSubscribe();
    }
}

bool PointCloudObjectFinder::FindObjectService(
    tinker_object_recognition::FindObjects::Request& req,
    tinker_object_recognition::FindObjects::Response& res) {
    if (!running_) StartSubscribe();
    service_running_ = true;
    while (!rgb_ready_ || !depth_ready_) {
        ros::spinOnce();
    }
    object_recognition_msgs::RecognizedObjectArray recognized_objects =
        GetRecognizedObjects();
    res.objects = recognized_objects;
    res.success = true;
    service_running_ = false;
    return true;
}

vector<PointCloudPtr> PointCloudObjectFinder::GetObjectPointClouds() {
    cv::resize(depth_image_, depth_image_, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(rgb_image_, rgb_image_, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    cv::Mat gray_image;
    cv::cvtColor(rgb_image_, gray_image, CV_BGR2GRAY);
    cv::Mat line_mask = LineFilterMask(gray_image, 3.5);
    ApplyMask(line_mask, depth_image_, cv::Vec3s(0, 0, 0));
    ApplyMask(line_mask, rgb_image_, cv::Vec3b(0, 0, 0));
    cv::Mat entropy_mask = EntropyFilterMask(gray_image, 0.4, 5, 2);
    DilateImage(entropy_mask, 4);
    ErodeImage(entropy_mask, 6);
    DilateImage(entropy_mask, 6);
    ApplyMask(entropy_mask, depth_image_, cv::Vec3s(0, 0, 0));
    ApplyMask(entropy_mask, rgb_image_, cv::Vec3b(0, 0, 0));
    PointCloudPtr filtered_cloud = BuildPointCloud(depth_image_, rgb_image_);
    ClusterDivider divider(filtered_cloud);
    return divider.GetDividedPointClouds();
}

object_recognition_msgs::RecognizedObjectArray
PointCloudObjectFinder::GetRecognizedObjects() {
    rgb_ready_ = false;
    depth_ready_ = false;
    vector<PointCloudPtr> object_clouds = GetObjectPointClouds();
    object_recognition_msgs::RecognizedObjectArray recognized_objects;
    std_msgs::Header header;
    header.seq = object_seq_++;
    header.stamp = ros::Time::now();
    header.frame_id = "base_link";
    recognized_objects.header = header;
    int i = 0;
    char buf[100];
    pcl::PointCloud<pcl::PointXYZRGB> debug_cloud_;
    ROS_INFO("found %lu objects", object_clouds.size());
    BOOST_FOREACH (PointCloudPtr object_cloud, object_clouds) {
        sprintf(buf, "/home/iarc/kinect_data/%d.pcd", i);
        pcl::io::savePCDFile(buf, *object_cloud);
        debug_cloud_ += *object_cloud;
        object_recognition_msgs::RecognizedObject object;
        geometry_msgs::Point center = GetCenter(object_cloud);
        ROS_INFO("center at %f %f %f", center.x, center.y, center.z);
        sensor_msgs::PointCloud2 cloud = ToROSCloud(*object_cloud);
        object.header = header;
        object.pose.header = header;
        object.point_clouds.push_back(cloud);
        object.pose.header = header;
        object.pose.pose.pose.position = center;
        recognized_objects.objects.push_back(object);
        i++;
    }
    sensor_msgs::PointCloud2 debug_cloud = ToROSCloud(debug_cloud_);
    debug_cloud.header.seq = debug_seq_++;
    debug_cloud.header.stamp = ros::Time::now();
    debug_cloud.header.frame_id = "base_link";
    debug_pub_.publish(debug_cloud);
    return recognized_objects;
}

void PointCloudObjectFinder::StartSubscribe() {
    depth_image_subscribe_ = nh.subscribe(
        depth_topic_name_, 10, &PointCloudObjectFinder::DepthCallback, this);
    rgb_image_subscribe_ = nh.subscribe(
        rgb_topic_name_, 10, &PointCloudObjectFinder::RGBCallback, this);
    depth_ready_ = false;
    rgb_ready_ = false;
    running_ = true;
}

void PointCloudObjectFinder::StopSubscribe() {
    if (running_) {
        depth_image_subscribe_.shutdown();
        rgb_image_subscribe_.shutdown();
    }
    depth_ready_ = false;
    rgb_ready_ = false;
    service_running_ = false;
    topic_running_ = false;
    running_ = false;
}
}
}

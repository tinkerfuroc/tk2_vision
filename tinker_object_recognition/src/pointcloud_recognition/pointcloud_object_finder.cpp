#include "tinker_object_recognition/pointcloud_recognition/pointcloud_object_finder.h"
#include "tinker_object_recognition/pointcloud_recognition/ClusterDivider.h"
#include "tinker_camera/pointcloud_builder.h"
#include "tinker_object_recognition/graph_filter/ColorFixer.h"
#include "tinker_object_recognition/graph_filter/filters.h"
#include "tinker_object_recognition/utilities.h"
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <cstdio>

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
    classify_client_ =
        nh.serviceClient<tinker_vision_msgs::ObjectClassify>("object_classify");
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
    if (!topic_running_ && !service_running_) return;
    mutex_.lock();
    ROS_INFO("Get depth");
    frame_id_ = depth_image->header.frame_id;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("depth image convert exception: %s", e.what());
    }
    depth_image_ = cv_ptr->image;
    depth_ready_ = true;
    mutex_.unlock();
}

void PointCloudObjectFinder::RGBCallback(
    const sensor_msgs::Image::ConstPtr& rgb_image) {
    if (!topic_running_ && !service_running_) return;
    mutex_.lock();
    ROS_INFO("Get RGB");
    frame_id_ = rgb_image->header.frame_id;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb_image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("depth image convert exception: %s", e.what());
    }
    rgb_image_ = cv_ptr->image;
    rgb_ready_ = true;
    mutex_.unlock();
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
    tinker_vision_msgs::FindObjects::Request& req,
    tinker_vision_msgs::FindObjects::Response& res) {
    ROS_INFO("Sevice called");
    mutex_.lock();
    ROS_INFO("Sevice lock acquired");
    service_running_ = true;
    mutex_.unlock();
    if (!running_) StartSubscribe();
    ros::Rate r(5);
    while (!rgb_ready_ || !depth_ready_) {
        r.sleep();
    }
    mutex_.lock();
    object_recognition_msgs::RecognizedObjectArray recognized_objects =
        GetRecognizedObjects();
    res.objects = recognized_objects;
    res.success = true;
    service_running_ = false;
    mutex_.unlock();
    return true;
}

vector<PointCloudPtr> PointCloudObjectFinder::GetObjectPointClouds() {
    cv::Mat rgb_image;
    cv::resize(depth_image_, depth_image_, cv::Size(), 0.5, 0.5);
    cv::resize(rgb_image_, rgb_image, cv::Size(), 0.5, 0.5);
    cv::Mat gray_image;
    cv::cvtColor(rgb_image, gray_image, CV_BGR2GRAY);
    //cv::Mat mask = LineFilterMask(gray_image, 3.5);
    //ApplyMask(mask, depth_image_, cv::Vec3s(0, 0, 0));
    //cv::imwrite("/home/iarc/line_mask.png", mask);

    cv::Mat mask = EntropyFilterMask(gray_image, 0.4, 5, 2);
    cv::imwrite("/home/iarc/entropy_mask.png", mask);
    //ApplyMask<unsigned char>(entropy_mask, mask, 0);

    cv::Mat depth_mask = DepthFilterMask(depth_image_, 0.3, 2.0);
    ApplyMask<unsigned char>(depth_mask, mask, 0);
    cv::imwrite("/home/iarc/depth_mask.png", mask);
    DilateImage(mask, 4);
    ErodeImage(mask, 6);
    DilateImage(mask, 6);
    cv::imwrite("/home/iarc/mask.png", mask);
    ApplyMask(mask, depth_image_, cv::Vec3s(0, 0, 0));

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    cv::findContours(mask, contours, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_SIMPLE);
    vector<PointCloudPtr> object_pointclouds;
    FILE *f = fopen("/home/iarc/object_names.txt", "w");
    int found_cnt = 0;
    for (int i = 0; i < contours.size(); i++) {
        cv::Rect boundrect = cv::boundingRect(cv::Mat(contours[i]));
        if (boundrect.width * boundrect.height < 400) continue;
        cv::Rect origin_rect = boundrect;
        origin_rect.x *= 2;
        origin_rect.y *= 2;
        origin_rect.height *= 2;
        origin_rect.width *= 2;
        cv::Mat object_img = rgb_image_(origin_rect);
        tinker_vision_msgs::ObjectClassify classify_srv;
        cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", object_img);
        cvi.toImageMsg(classify_srv.request.img);
        if (classify_client_.call(classify_srv)) {
            if (classify_srv.response.found &&
                classify_srv.response.df_val < -0.2 &&
                classify_srv.response.name.data.find("fuck") != 0) {
                cv::rectangle(rgb_image, cv::Point(boundrect.x, boundrect.y),
                              cv::Point(boundrect.x + boundrect.width,
                                        boundrect.y + boundrect.height),
                              cv::Scalar(0, 255, 0));
                char label_text[200];
                sprintf(label_text, "%d", found_cnt);
                fprintf(f, "%d %s\n", found_cnt, classify_srv.response.name.data.c_str());
                found_cnt++;
                cv::putText(rgb_image, label_text,
                        cv::Point(boundrect.x, boundrect.y), 
                        cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0));
                object_pointclouds.push_back(BuildPointCloud(
                    depth_image_(boundrect), rgb_image(boundrect)));
            } else {
                cv::rectangle(rgb_image, cv::Point(boundrect.x, boundrect.y),
                              cv::Point(boundrect.x + boundrect.width,
                                        boundrect.y + boundrect.height),
                              cv::Scalar(0, 0, 255));
            }
        }
    }
    fclose(f);
    cv::imwrite("/home/iarc/result.png", rgb_image);
    return object_pointclouds;
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
    header.frame_id = "kinect_kinect2_rgb_link";
    recognized_objects.header = header;
    int i = 0;
    char buf[100];
    pcl::PointCloud<pcl::PointXYZRGB> debug_cloud_;
    ROS_INFO("found %lu objects", object_clouds.size());
    BOOST_FOREACH (PointCloudPtr object_cloud, object_clouds) {
        if (object_cloud->width < 100) continue;
        sprintf(buf, "/home/iarc/kinect_data/%d.pcd", i);
        // pcl::io::savePCDFile(buf, *object_cloud);
        debug_cloud_ += *object_cloud;
        object_recognition_msgs::RecognizedObject object;
        geometry_msgs::Point center = GetCenter(object_cloud);
        ROS_INFO("size %d, center at %f %f %f", object_cloud->width, center.x,
                 center.y, center.z);
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
    debug_cloud.header.frame_id = header.frame_id;
    debug_pub_.publish(debug_cloud);
    return recognized_objects;
}

void PointCloudObjectFinder::StartSubscribe() {
    mutex_.lock();
    depth_image_subscribe_ = nh.subscribe(
        depth_topic_name_, 10, &PointCloudObjectFinder::DepthCallback, this);
    rgb_image_subscribe_ = nh.subscribe(
        rgb_topic_name_, 10, &PointCloudObjectFinder::RGBCallback, this);
    depth_ready_ = false;
    rgb_ready_ = false;
    running_ = true;
    mutex_.unlock();
}

void PointCloudObjectFinder::StopSubscribe() {
    ros::Rate r(1);
    while (service_running_) r.sleep();
    mutex_.lock();
    if (running_) {
        depth_image_subscribe_.shutdown();
        rgb_image_subscribe_.shutdown();
    }
    depth_ready_ = false;
    rgb_ready_ = false;
    service_running_ = false;
    topic_running_ = false;
    running_ = false;
    mutex_.unlock();
}
}
}

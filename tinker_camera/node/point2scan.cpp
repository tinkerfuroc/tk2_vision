#include "tinker_camera/pointcloud_builder.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace tinker::vision;
using std::string;
using std::vector;

void ToPolar(double x, double y, double &r, double &theta) {
    double dis = sqrt(x * x + y * y);
    theta = acos(x / dis);
    if (y < 0) theta = -theta;
    r = dis;
}

class PointCloud2Scan {
public:
    PointCloud2Scan()
        : nh_(),
          private_nh_("~"),
          rgb_ready_(false),
          depth_ready_(false),
          seq_(0) {
        private_nh_.param("depth_topic_name", depth_topic_name_,
                          string("/head/kinect2/depth/image_depth"));
        private_nh_.param("rgb_topic_name", rgb_topic_name_,
                          string("/head/kinect2/rgb/image_color"));
        private_nh_.param("frame_id", frame_id_,
                          string("kinect_kinect2_rgb_link"));
        pointcloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/kinect_pointcloud", 1);
        scan_pub_ =
            nh_.advertise<sensor_msgs::LaserScan>("/kinect_fake_laser", 1);
        rgb_sub_ = nh_.subscribe(rgb_topic_name_, 1,
                                 &PointCloud2Scan::RGBCallback, this);
        depth_sub_ = nh_.subscribe(depth_topic_name_, 1,
                                   &PointCloud2Scan::DepthCallback, this);
    }

    void DepthCallback(const sensor_msgs::Image::ConstPtr &depth_image) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_image);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("depth image convert exception: %s", e.what());
        }
        depth_image_ = cv_ptr->image;
        depth_ready_ = true;
    }

    void RGBCallback(const sensor_msgs::Image::ConstPtr &rgb_image) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(rgb_image);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("depth image convert exception: %s", e.what());
        }
        rgb_image_ = cv_ptr->image;
        rgb_ready_ = true;
    }

    void PublishData() {
        if (!rgb_ready_ || !depth_ready_) return;
        seq_++;
        rgb_ready_ = false;
        depth_ready_ = false;
        PointCloudPtr pointcloud = BuildPointCloud(depth_image_, rgb_image_);
        sensor_msgs::PointCloud2 ros_cloud = ToROSCloud(*pointcloud);
        ros_cloud.header.seq = seq_;
        ros_cloud.header.frame_id = frame_id_;
        ros_cloud.header.stamp = ros::Time::now();
        pointcloud_pub_.publish(ToROSCloud(*pointcloud));
        PublishLaserScan(pointcloud);
    }

private:
    void PublishLaserScan(PointCloudPtr pointcloud) {
        sensor_msgs::LaserScan laser_scan;
        static const double angle_increment = 0.005;
        laser_scan.range_min = 0;
        laser_scan.range_max = 10;
        laser_scan.angle_min = -M_PI;
        laser_scan.angle_max = M_PI;
        laser_scan.angle_increment = angle_increment;
        int num_points = int(2 * M_PI / angle_increment) + 1;
        vector<float> ranges(num_points, INFINITY);
        for (int i = 0; i < pointcloud->points.size(); i++) {
            const pcl::PointXYZRGB &p = pointcloud->points[i];
            double x = p.x;
            double y = p.y;
            double r, theta;
            ToPolar(x, y, r, theta);
            int p_index = (theta + M_PI) / angle_increment;
            p_index = p_index < 0 ? 0 : p_index;
            p_index = p_index >= num_points ? num_points - 1 : p_index;
            ranges[p_index] = ranges[p_index] < r ? ranges[p_index] : r;
        }
        laser_scan.ranges = ranges;
        laser_scan.header.seq = seq_;
        laser_scan.header.frame_id = frame_id_;
        laser_scan.header.stamp = ros::Time::now();
        scan_pub_.publish(laser_scan);
    }

    void PublishPointcloud(PointCloudPtr pointcloud);
    string rgb_topic_name_;
    string depth_topic_name_;
    string frame_id_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher scan_pub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    bool rgb_ready_;
    bool depth_ready_;
    int seq_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "point2scan");
    PointCloud2Scan p2s;
    ros::Rate r(10.);
    while(ros::ok()) {
        p2s.PublishData();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

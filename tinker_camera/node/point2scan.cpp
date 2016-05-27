#include "tinker_camera/pointcloud_builder.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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
          it_(nh_),
          seq_(0) {
        private_nh_.param("depth_topic_name", depth_topic_name_,
                          string("/head/kinect2/depth/image_depth"));
        private_nh_.param("frame_id", frame_id_,
                          string("kinect_kinect2_rgb_link"));
        pointcloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/kinect_pointcloud", 1);
        scan_pub_ =
            nh_.advertise<sensor_msgs::LaserScan>("/kinect_fake_laser", 1);
        depth_sub_ = it_.subscribe(depth_topic_name_, 1,
                                   &PointCloud2Scan::DepthCallback, this);
        ROS_INFO("[Point2Laser] Init Done");
    }

    void DepthCallback(const sensor_msgs::ImageConstPtr &depth_image) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_image, "16SC3");
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("depth image convert exception: %s", e.what());
        }
        depth_image_ = cv_ptr->image;
        PublishData();
    }

    void PublishData() {
        seq_++;
        PointCloudPtr pointcloud = BuildPointCloud(depth_image_);
        sensor_msgs::PointCloud2 ros_cloud = ToROSCloud(*pointcloud);
        ros_cloud.header.seq = seq_;
        ros_cloud.header.frame_id = frame_id_;
        ros_cloud.header.stamp = ros::Time::now();
        pointcloud_pub_.publish(ros_cloud);
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
            //ROS_INFO("at %f %f %f", p.x, p.y, p.z);
            int p_index = (theta + M_PI) / angle_increment;
            p_index = p_index < 0 ? 0 : p_index;
            p_index = p_index >= num_points ? num_points - 1 : p_index;
            ranges[p_index] = ranges[p_index] < r ? ranges[p_index] : r;
            //ROS_INFO("%d: %f(%f) ", p_index, ranges[p_index], r);
        }
        laser_scan.ranges = ranges;
        laser_scan.header.seq = seq_;
        laser_scan.header.frame_id = frame_id_;
        laser_scan.header.stamp = ros::Time::now();
        scan_pub_.publish(laser_scan);
    }

    string depth_topic_name_;
    string frame_id_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher scan_pub_;
    image_transport::Subscriber depth_sub_;
    cv::Mat depth_image_;
    int seq_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "point2scan");
    PointCloud2Scan p2s;
    ros::Rate r(10.);
    ros::spin();
    return 0;
}

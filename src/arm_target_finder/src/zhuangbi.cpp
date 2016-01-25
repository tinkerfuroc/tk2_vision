//demo node.
//subscribe tk2_vision/arm_target_finder/caught_obj and decode it.
//Du 2016.1.15.
//

#include <arm_target_finder/ForegroundDetector.h>
#include <arm_target_finder/TargetFound.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <vector>
#include <string>

void CaughtImgCallback(const arm_target_finder::TargetFound::ConstPtr& msg);

image_transport::Publisher pub;

void CaughtImgCallback(const arm_target_finder::TargetFound::ConstPtr& msg)
{
    //get foreground
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->origin_img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cam_mat = cv_ptr->image;
    if(msg->object_recognized)
    {
        cv::Rect rect(msg->object_rect_left, msg->object_rect_top, msg->object_rect_width, msg->object_rect_height);
        cv::rectangle(cam_mat, rect, cv::Scalar(0, 0, 255));
        ROS_INFO("find %s, probability %f", msg->object_name.c_str(), msg->object_similarity);
    }
    else
    {
        ROS_INFO("find nothing");
    }
    //publish it
    sensor_msgs::Image pubmsg;
    cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", cam_mat);
    cvi.toImageMsg(pubmsg);
    pub.publish(pubmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_target_finder_with_zhuangbility");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pub = it.advertise("tk2_vision/arm_target_finder/zhuangbi", 1);
    ros::Subscriber sub = nh.subscribe("tk2_vision/arm_target_finder/caught_obj", 1, CaughtImgCallback);
    ros::spin();
    return 0;
}

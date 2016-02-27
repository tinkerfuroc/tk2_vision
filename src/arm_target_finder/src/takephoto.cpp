//main node.
//catch tk2_com/arm_cam_image and find fucking objects out.
//Du 2016.1.15.
//

#include <arm_target_finder/ForegroundDetector.h>
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

void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr& msg);

tinker::vision::ForegroundDetector fd(8, 0.265, 0);
image_transport::Publisher pub;
void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //get foreground
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat cam_mat = cv_ptr->image;
    cv::Mat res_mat = fd.TakePhoto(cam_mat);
    if (res_mat.rows > 0 && res_mat.cols > 0)
    {
        //publish it
        sensor_msgs::Image pubmsg;
        cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", res_mat);
        cvi.toImageMsg(pubmsg);
        pub.publish(pubmsg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_webcam_img_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pub = it.advertise("camera/image", 1);
    ros::Subscriber sub = nh.subscribe("tk2_com/arm_cam_image", 1, ArmCamImgCallback);
    ros::spin();
    return 0;
}

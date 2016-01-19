//main node.
//catch tk2_com/arm_cam_image and find fucking objects out.
//Du 2016.1.15.
//

#include <arm_target_finder/ForegroundDetector.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <vector>
#include <string>

void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr& msg);

tinker::vision::ForegroundDetector fd(8, 0.265, 0.73);
image_transport::Publisher pub;

static const int ImageTypeNum = 6;
static const char *ImageTypeName[] = 
{
    //"dianchi", 
    "bitong",
    "akarrin",
    "alps",
    "apple",
    "bingtangxueli",
    "pingzi",
    //"tsinghua",
};
static const cv::Scalar ImageTypeRectColor[] = 
{
    //cv::Scalar(0, 0, 255), 
    cv::Scalar(0, 255, 0), 
    cv::Scalar(255, 0, 0), 
    cv::Scalar(255, 255, 0), 
    cv::Scalar(0, 255, 255),
    cv::Scalar(255, 0, 255),
    cv::Scalar(0, 0, 0), 
    //cv::Scalar(255, 255, 255), 
};
static const int ImageTypePhotonum[] = 
{
    //4,
    7,
    4,
    3,
    2,
    4,
    2,
    //3,
};
tinker::vision::ZhuangBiObjectInfo **objs;

int sumofimages = 0;


int InitObjs()
{
    char imagefilename[50];
    char imagename[50];
    for (int i=0; i<ImageTypeNum; i++)
    {
        sumofimages += ImageTypePhotonum[i];
    }
    objs = new tinker::vision::ZhuangBiObjectInfo*[sumofimages];
    int c = 0;
    for (int i=0; i<ImageTypeNum; i++)
    {
        for (int j=0; j<ImageTypePhotonum[i]; j++)
        {
            sprintf(imagefilename, "%s%d.png", ImageTypeName[i], j+1);
            sprintf(imagename, "%s%d", ImageTypeName[i], j+1);
            cv::Mat frame = cv::imread(imagefilename, CV_LOAD_IMAGE_COLOR);
            if(!frame.data)                              // Check for invalid input
            {
                ROS_ERROR("Could not open or find the image %s", imagefilename);
                return 0;
            }
            objs[c] = new tinker::vision::ZhuangBiObjectInfo(frame, std::string(imagename), ImageTypeRectColor[i]);
            c++;
        }
    }
    return 1;
}

void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //get foreground
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat cam_mat = cv_ptr->image;
    cv::Mat res_mat;
    cam_mat.copyTo(res_mat);
    //fd.Filter(cam_mat, res_mat);
    fd.DrawRectOnDetect(cam_mat, res_mat, objs, sumofimages);
    //publish it
    sensor_msgs::Image pubmsg;
    cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", res_mat);
    cvi.toImageMsg(pubmsg);
    pub.publish(pubmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_webcam_img_detector");
    ros::NodeHandle nh;
    if (!InitObjs())
    {
        return 1;
    }
    image_transport::ImageTransport it(nh);
    pub = it.advertise("camera/image", 1);
    ros::Subscriber sub = nh.subscribe("tk2_com/arm_cam_image", 1, ArmCamImgCallback);
    ros::spin();
    return 0;
}

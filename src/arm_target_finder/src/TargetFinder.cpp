//main node for tk2.
//catch tk2_com/arm_cam_image and find fucking objects out.
//
//input: camera image(sensor_msgs/Image)tk2_com/arm_cam_image
//output:caught object(arm_target_finder/TargetFound) tk2_vision/arm_target_finder/caught_obj
//
//Du 2016.1.15.
//

#include <arm_target_finder/ForegroundDetector.h>
#include <arm_target_finder/TargetFound.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <unistd.h>

void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr& msg);

tinker::vision::ForegroundDetector fd(8, 0.265, 0.73);
ros::Publisher pub;
using std::string;

static const int ImageTypeNum = 4;
static const char *ImageTypeName[] = 
{
    //"dianchi", 
    //"coffee",
    //"akarrin",
    "cola",
    "bottle"
    //"tsinghua",
};
static const cv::Scalar ImageTypeRectColor[] = 
{
    //cv::Scalar(0, 0, 255), 
    //cv::Scalar(0, 255, 0), 
    cv::Scalar(255, 0, 0), 
    cv::Scalar(255, 255, 0), 
    cv::Scalar(0, 255, 255),
    cv::Scalar(255, 0, 255),
    //cv::Scalar(0, 0, 0), 
    //cv::Scalar(255, 255, 255), 
};

static const int ImageTypePhotonum[] = 
{
    //4,
    //5,
    8,
    5,
    //2,
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
            sprintf(imagefilename, "/home/furoc/tinker/data/%s%d.png", ImageTypeName[i], j+1);
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
    
    arm_target_finder::TargetFound tf;
    cv::Rect pos;
    const char *objectname;
    bool object_recognized;
    float object_similarity;
    fd.FindObject(cam_mat, object_recognized, object_similarity, 
        objectname, pos, objs, sumofimages);
    tf.object_similarity = object_similarity;
    tf.object_recognized = object_recognized;
    tf.object_name = std::string(objectname);
    tf.object_rect_left = pos.x;
    tf.object_rect_top = pos.y;
    tf.object_rect_width = pos.width;
    tf.object_rect_height = pos.height;
    tf.origin_img = *msg;
    tf.header = std_msgs::Header();
    //publish it
    pub.publish(tf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_target_finder");
    ros::NodeHandle nh;
    if (!InitObjs())
    {
        return 1;
    }
    pub = nh.advertise<arm_target_finder::TargetFound>("tk2_vision/arm_target_finder/caught_obj", 1);
    ros::Subscriber sub = nh.subscribe("tk2_com/arm_cam_image", 1, ArmCamImgCallback);
    ros::spin();
    return 0;
}

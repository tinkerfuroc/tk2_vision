#include <cstdio>
#include <cstdlib>
#include <string>
#include <tinker_object_recognition/SRC_model/SparseRecognition.h>
#include <tinker_object_recognition/arm_target_finder/ForegroundDetector.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <vector>
#include <cassert>

using std::string;

void ArmCamImgCallback(const sensor_msgs::Image::ConstPtr& msg);
tinker::vision::ForegroundDetector fd(8, 0.265);
int Usage();

image_transport::Publisher pub;

const char *filename = "fuck.png";
string model_filename;
SRCModel *src_model;


int main(int argc, char * argv[])
{
    if(argc != 2)
        return Usage();
    model_filename = string(argv[1]);
    
    src_model = LoadSRCModel(model_filename);
    ros::init(argc, argv, "arm_webcam_object_classifier");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pub = it.advertise("tk2_com/aci_fnd", 1);
    ros::Subscriber sub = nh.subscribe("tk2_com/arm_cam_image", 1, ArmCamImgCallback);
    ros::spin();
	ReleaseSRCModel(&src_model);	
    return 0;
}


int Usage()
{
    printf("usage: object_classify model_file\n");
    return 0;
}



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
        
        //run src model to recognize it
        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);   
        imwrite(filename, res_mat, compression_params);
        CvMat *y = LoadSample(filename, src_model->sample_size_);
        string name;
        double sci;
        Recognize(src_model, y, 0.1, sci, name, NULL, NULL);
        printf("sci: %f, name: %s\n", sci, name.c_str());
    }
    
}

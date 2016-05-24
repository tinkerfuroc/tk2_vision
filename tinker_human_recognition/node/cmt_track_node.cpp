#include "tinker_human_recognition/cmt_track/CMT.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <getopt.h>
#include <ros/ros.h>
#include <k2_client/BodyArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class CMT_track_node
{
public:
    void ImageCallBack(const sensor_msgs::Image::ConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cam_mat = cv_ptr->image;
        if (cam_mat.empty()) return;
        if (init_done_)
        {
            cv::Mat cam_mat_gray;
            if (cam_mat.channels() > 1) {
                cv::cvtColor(cam_mat, cam_mat_gray, CV_BGR2GRAY);
            } else {
                cam_mat_gray = cam_mat;
            }
            //Let CMT process the frame
            cmt_.processFrame(cam_mat_gray);
            
            //compare histogram using earth mover's distance
            cv::Rect rect = getRectFromCMT();
            cv::Mat hist;
            getHist(cam_mat, rect, hist);
            float emd = cv::EMD(hist, start_histogram_, CV_DIST_L1);
            
            is_same_ = (emd < emd_threshold_);
            
            //debug image display
            Draw(cam_mat);
            sensor_msgs::Image img;
            cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", cam_mat);
            cvi.toImageMsg(img);
            dbg_pub_.publish(img);
            // if dead
            if (0)
            {
                init_done_ = false;
            }
        }
        else
        {
            start_mat_ = cam_mat;
            ROS_WARN("no body detected.");
        }
    }
    
    void K2BodyCallBack(const k2_client::BodyArray::ConstPtr &msg) {
        if (!init_done_ && !start_mat_.empty()) {
            k2_client::Body body = msg->bodies[0];
            start_rect_ = cv::Rect(min(body.toX, body.fromX), min(body.toY, body.fromY),
                abs(body.toX - body.fromX), abs(body.toY - body.fromY));
            Mat img_gray;
            if (start_mat_.channels() > 1) {
                cv::cvtColor(start_mat_, img_gray, CV_BGR2GRAY);
            } else {
                img_gray = start_mat_;
            }
            cmt_.initialize(img_gray, start_rect_);
            init_done_ = true;
            
            getHist(start_mat_, start_rect_, start_histogram_);
            ROS_INFO("kinect body detected. cmt start.");
        }
    }
    
    void getHist(cv::Mat &source, cv::Rect &bound, cv::Mat &desk)
    {
        cv::Rect rect(bound.x, bound.y, bound.width, min(bound.height, source.rows - bound.y - 1));
        cv::Mat roi(source, rect);
        cv::Mat norm_mat;
        cv::resize(roi, norm_mat, cv::Size(100, 100));
        
        const int imgCount = 1;
        const int dims = 3;
        const int sizes[] = {256,256,256};
        const int channels[] = {0,1,2};
        const float rRange[] = {0,256};
        const float gRange[] = {0,256};
        const float bRange[] = {0,256};
        const float *ranges[] = {rRange,gRange,bRange};
        Mat mask = Mat();
        
        calcHist(&norm_mat, imgCount, channels, mask, desk, dims, sizes, ranges);
    }
    
    cv::Rect getRectFromCMT()
    {
        /*
        cv::Point2f vertices[4];
        cmt_.bb_rot.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            cv::line(im, vertices[i], vertices[(i+1)%4], cv::Scalar(255,0,0));
        }
        */
        
        //it seems that the bb_rot is a zhuangbi rotatedrect actually rect.
        //this B has the technique.
        return cmt_.bb_rot.boundingRect();
    }
    
    void Draw(cv::Mat &im)
    {
        //draw points
        for(size_t i = 0; i < cmt_.points_active.size(); i++)
        {
            cv::circle(im, cmt_.points_active[i], 2, cv::Scalar(255,0,0));
        }
        //draw rect
        if(is_same_)
        {
            cv::rectangle(im, getRectFromCMT(), cv::Scalar(255,255,255));
            ROS_INFO("it seems that the histograms are the same");
        }
        else
        {
            cv::rectangle(im, getRectFromCMT(), cv::Scalar(255,0,0));
            ROS_INFO("it seems that the histograms are the same");
        }    
            
    }

    CMT_track_node()
        : init_done_(false), emd_threshold_(0.5)
    {
        img_sub_ = nh_.subscribe("head/kinect2/rgb/image_color", 1,
            &CMT_track_node::ImageCallBack, this);
        k2_body_sub_ = nh_.subscribe("head/kinect2/bodyArray", 1,
            &CMT_track_node::K2BodyCallBack, this);
        dbg_pub_ = nh_.advertise<sensor_msgs::Image>("tk2_vision/dbg_cmtimg", 1);
    }
    
    
protected:
    bool is_same_;

    cmt::CMT cmt_;
    cv::Rect start_rect_;
    cv::Mat start_mat_;
    cv::Mat start_histogram_;
    bool init_done_;
    
    float emd_threshold_;
    
    ros::NodeHandle nh_;
    ros::Subscriber img_sub_;
    ros::Subscriber k2_body_sub_;
    ros::Publisher dbg_pub_;
    
    int abs(int n) {return n>0?n:-n;}
    int min(int a, int b) {return a>b?b:a;}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmt_track_node");
    CMT_track_node n;
    ros::spin();
    return 0;
}


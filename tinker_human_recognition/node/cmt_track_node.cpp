#include "tinker_human_recognition/cmt_track/CMT.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <vector>
#include <getopt.h>
#include <ros/ros.h>
#include <k2_client/BodyArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using std::vector;

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
            cv::cvtColor(cam_mat, cam_mat_gray, CV_BGR2GRAY);
            
            try
            {
                //Let CMT process the frame
                cmt_.processFrame(cam_mat_gray);
            }
            catch (...)
            {
                // mdzz why can I get a nan error in fastclauster
                // dirty fuck
                ROS_WARN("get strange error in cmt::processframe");
                return;
            }
            
            //compare histogram using earth mover's distance
            cv::Rect rect = getRectFromCMT();
            cv::Mat hist = getHist(cam_mat, rect);
            float emd = compare_histograms(hist, start_histogram_);
            ROS_INFO("emd: %f", emd);
            is_same_ = (emd < emd_threshold_);
            
            //debug image display
            Draw(cam_mat);
            
            // if dead
            if (!is_same_ )
            {
            //    init_done_ = false;
                ROS_WARN("lost sight of human.");
            }
        }
        else
        {
            start_mat_ = cam_mat;
            ROS_WARN("no body detected.");
        }
        sensor_msgs::Image img;
        cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", cam_mat);
        cvi.toImageMsg(img);
        dbg_pub_.publish(img);
    }
    

    void K2BodyCallBack(const k2_client::BodyArray::ConstPtr &msg) 
    {
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
            cmt_ = cmt::CMT();
            cmt_.initialize(img_gray, start_rect_);
            init_done_ = true;
            
            start_histogram_ = getHist(start_mat_, start_rect_);
            ROS_INFO("kinect body detected. cmt start.");
        }
    }
    
    cv::Mat getHist(cv::Mat &source, cv::Rect &bound) 
    {
        int rectupY = max(bound.y, 0);
        int rectdownY = min(source.rows, bound.y+bound.height);
        int rectleftX = max(bound.x, 0);
        int rectrightX = min(source.cols, bound.x+bound.width);
        ROS_INFO("fuck %d %d %d %d", rectleftX, rectupY, rectrightX - rectleftX, rectdownY - rectupY);
        cv::Rect rect(rectleftX, rectupY, rectrightX - rectleftX, rectdownY - rectupY);
        
        cv::Mat roi(source, rect);
        cv::Mat norm_mat;
        cv::resize(roi, norm_mat, cv::Size(100, 100));
        
	    const int dims = 1;
	    const int sizes[] = { 256, 256, 256};
	    const int num_channels = 3;
	    const int channels[] = { 0, 1, 2 };
	    float rRange[] = { 0, 256 };
	    float gRange[] = { 0, 256 };
	    float bRange[] = { 0, 256 };
	    const float *ranges[] = { rRange, gRange, bRange };
	    cv::Mat mask = cv::Mat();

	    cv::Mat hist = cv::Mat(0, 1, CV_32F);
	    cv::Mat temp;
	    for (int c = 0; c < num_channels; c++) {
		    // Calculate the histogram for the current color channel
		    calcHist(&norm_mat, 1, &channels[c], mask, temp, dims, &sizes[c], &ranges[c], true, false);
		    // Append the current channel's histogram
		    hist.push_back(temp);
		    //cout << hist;
	    }
	    // Transpose the histogram so we have a row vector
	    cv::Mat hist_t = hist.clone().t();
	    hist.release();

	    // Normalize the histogram
	    normalize(hist_t, hist_t, 1.0, 0.0, cv::NORM_L1);
	    return hist_t;
    }

    float compare_histograms(cv::Mat hist1, cv::Mat hist2) 
    {
	    assert(hist1.cols == hist2.cols);

	    // Make a copy of the histograms as we will be adding columns
	    cv::Mat hist1copy = hist1.t();
	    cv::Mat hist2copy = hist2.t();

	    // Append an index of the corresponding row of each histogram bin, for EMD
	    cv::Mat emd_cols = cv::Mat::zeros(hist1.cols, 1, CV_32F);
	    for (int i = 0; i < hist1.cols; i++) {
		    emd_cols.at<float>(i, 0) = i + 1;
	    }

	    hconcat(hist1copy, emd_cols, hist1copy);
	    hconcat(hist2copy, emd_cols, hist2copy);

	    return EMD(hist1copy, hist2copy, CV_DIST_L1);
    }
    
    
    cv::Rect getRectFromCMT()
    {
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
            cv::rectangle(im, getRectFromCMT(), cv::Scalar(255,255,255), 3);
        }
        else
        {
            cv::rectangle(im, getRectFromCMT(), cv::Scalar(255,0,0), 3);
        }    
            
    }

    CMT_track_node()
        : init_done_(false), emd_threshold_(20)
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
    int max(int a, int b) {return a<b?b:a;}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmt_track_node");
    CMT_track_node n;
    ros::spin();
    return 0;
}


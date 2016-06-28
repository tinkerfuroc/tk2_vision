#include "tinker_human_recognition/cmt_track/CMT.h"
#include "tinker_object_recognition/utilities.h"
#include "tinker_object_recognition/graph_filter/filters.h"
#include "tinker_camera/pointcloud_builder.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tinker_vision_msgs/EmptyAction.h>

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
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>
using std::vector;

class CMT_track_node
{
public:

    bool isCenterValid(cv::Mat cam_mat_masked, geometry_msgs::Point &center) {
        printf("ofuck");
        center = getZeroPoint();
        if (cam_mat_masked.empty() || k2_depth_mat_.empty()) 
            return false;
        tinker::vision::PointCloudPtr pcp = 
            tinker::vision::BuildPointCloud(k2_depth_mat_, cam_mat_masked);
        center = tinker::vision::GetCenter(pcp);
        ROS_INFO("find at %lf %lf %lf, %d", center.x, center.y, center.z, pcp->width);
        return (pcp->width > pointcloud_width_tolerance_);
    }

    void ImageCallBack(const sensor_msgs::Image::ConstPtr &msg) {
        if(enable_)
        {
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat cam_mat = cv_ptr->image;
            cv::resize(cam_mat, cam_mat, cv::Size(),  1/resize_p_, 1/resize_p_);
            if (cam_mat.empty() || depth_mask_.empty()) 
                return;
            cv::Mat cam_mat_masked;
            cam_mat.copyTo(cam_mat_masked, depth_mask_);
            if (init_done_)
            {
                cv::Mat cam_mat_gray;
                cv::cvtColor(cam_mat_masked, cam_mat_gray, CV_BGR2GRAY);
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
                cv::Mat hist = getHist(cam_mat_masked, rect);
                float emd = compare_histograms(hist, start_histogram_);
                ROS_INFO("emd: %f", emd);
                is_same_ = (emd < emd_threshold_);
                
                //debug image display
                Draw(cam_mat); 
                bool find = false;
                geometry_msgs::Point center = getZeroPoint();
                find = is_same_ && (isCenterValid(cam_mat_masked, center));
                center_pub_.publish(center);
                if(find)
                {
                    lost_sight_cnt_ = 0;    
                }
                else
                {
                    lost_sight_cnt_++;
                    if(lost_sight_cnt_ >= lost_sight_tolerance_)
                    {
                        ROS_WARN("lost sight of human.");
                    }
                }
            }
            else
            {
                start_mat_masked_ = cam_mat_masked;
                start_mat_ = cam_mat;
                //ROS_WARN("no body detected.");
            }
            sensor_msgs::Image img;
            cv_bridge::CvImage cvi(std_msgs::Header(), "bgr8", cam_mat);
            cvi.toImageMsg(img);
            dbg_pub_.publish(img);
        }
    }
    

    void K2BodyCallBack(const k2_client::BodyArray::ConstPtr &msg) 
    {
        if (enable_ && !init_done_ && !start_mat_.empty() && !k2_depth_mat_.empty()) {
            k2_client::Body body = msg->bodies[0];
            int fromX = min(body.fromX, body.toX) / resize_p_;
            int toX = max(body.fromX, body.toX) / resize_p_ ;
            int fromY = min(body.fromY, body.toY) / resize_p_;
            int toY = max(body.fromY, body.toY) / resize_p_;
            fromX = max(fromX, 0);
            toX = min(toX, start_mat_.cols);
            fromY = max(fromY, 0);
            toY = min(toY, start_mat_.rows);
            start_rect_ = cv::Rect(fromX, fromY, toX-fromX, toY-fromY);
            //ROS_INFO("fuck1 %d %d %d %d", start_rect_.x, start_rect_.y, start_rect_.height, start_rect_.width);
            //ROS_INFO("fuck11 %d %d %d %d",  start_mat_masked_.rows, start_mat_masked_.cols, k2_depth_mat_.rows, k2_depth_mat_.cols);
            cv::Mat start_mat_masked(start_mat_masked_, start_rect_);
            cv::Mat k2_depth_mat(k2_depth_mat_, start_rect_);
            start_mat_masked.copyTo(start_mat_masked_);
            k2_depth_mat.copyTo(k2_depth_mat_);
            geometry_msgs::Point center;
            if(isCenterValid(start_mat_masked_, center))
            {
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
                enable_as_.setSucceeded();
            }
        }
    }
    
    void K2DepthCallBack(const sensor_msgs::Image::ConstPtr &msg) {
        if(enable_)
        {
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg);
            cv::Mat k2_depth_mat = cv_ptr->image;    //16SC3, so no encoding param is passed
            cv::resize(k2_depth_mat, k2_depth_mat, cv::Size(), 1/resize_p_, 1/resize_p_);
            depth_mask_ = tinker::vision::DepthFilterMask(k2_depth_mat, min_depth_, max_depth_);
            tinker::vision::DilateImage(depth_mask_, 3);
            tinker::vision::ErodeImage(depth_mask_, 6);
            tinker::vision::DilateImage(depth_mask_, 8);

            k2_depth_mat.copyTo(k2_depth_mat_, depth_mask_); 
        }
    }

    cv::Mat getHist(cv::Mat &source, cv::Rect &bound) 
    {
        int rectupY = max(bound.y, 0);
        int rectdownY = min(source.rows, bound.y+bound.height);
        int rectleftX = max(bound.x, 0);
        int rectrightX = min(source.cols, bound.x+bound.width);
        //ROS_INFO("fuck %d %d %d %d", rectleftX, rectupY, rectrightX - rectleftX, rectdownY - rectupY);
        cv::Rect rect(rectleftX, rectupY, rectrightX - rectleftX, rectdownY - rectupY);
        cv::Mat roi(source, rect);
        cv::Mat norm_mat;
        cv::resize(roi, norm_mat, cv::Size(100, 100));
        norm_mat = tinker::vision::HistogramEqualizeRGB(norm_mat);
        
	    const int dims = 1;
	    const int sizes[] = { 64, 64, 64};
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
        : private_nh_("~"), init_done_(false), resize_p_(8.0), pointcloud_width_tolerance_(500),
        lost_sight_cnt_(0), enable_(false),
        enable_as_(nh_, "tk2_vision/hum_recog_enable", false),
        disable_as_(nh_, "tk2_vision/hum_recog_disable", false)
    {
        XmlRpc::XmlRpcValue CMT_track_params;
        private_nh_.getParam("CMT_track_params", CMT_track_params);
        ROS_ASSERT(CMT_track_params.size() > 0);
        ROS_ASSERT(CMT_track_params.hasMember("emd_threshold"));
        double f = (double)CMT_track_params["emd_threshold"];
        emd_threshold_ = (float) f;
        ROS_ASSERT(CMT_track_params.hasMember("min_depth"));
        f = (double)CMT_track_params["min_depth"];
        min_depth_ = (float) f;
        ROS_ASSERT(CMT_track_params.hasMember("max_depth"));
        f = (double)CMT_track_params["max_depth"];
        max_depth_ = (float) f;
        ROS_ASSERT(CMT_track_params.hasMember("lost_sight_tolerance"));
        lost_sight_tolerance_ = (int)CMT_track_params["lost_sight_tolerance"];
        
        img_sub_ = nh_.subscribe("head/kinect2/rgb/image_color", 1,
            &CMT_track_node::ImageCallBack, this);
        k2_body_sub_ = nh_.subscribe("head/kinect2/bodyArray", 1,
            &CMT_track_node::K2BodyCallBack, this);
        k2_depth_sub_ = nh_.subscribe("head/kinect2/depth/image_depth", 1,
            &CMT_track_node::K2DepthCallBack, this);
        dbg_pub_ = nh_.advertise<sensor_msgs::Image>("tk2_vision/dbg_cmtimg", 1);
        center_pub_ = nh_.advertise<geometry_msgs::Point>("tk2_vision/human_center", 1);
        //hum_recog_enable_srv_ = nh_.advertiseService("tk2_vision/hum_recog_enable", 
        //    &CMT_track_node::humRecogEnable, this);
        //hum_recog_disable_srv_ = nh_.advertiseService("tk2_vision/hum_recog_disable", 
        //    &CMT_track_node::humRecogDisable, this);
        enable_as_.registerGoalCallback(boost::bind(&CMT_track_node::humRecogEnable, this));
        disable_as_.registerGoalCallback(boost::bind(&CMT_track_node::humRecogDisable, this));
        try
        {
            enable_as_.start();
            disable_as_.start();
        }
        catch(...)
        {
            ROS_ERROR("aclib start failed TAT");
        }
    }
    
    //bool humRecogEnable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    void humRecogEnable()
    {
      enable_ = true;
      ROS_INFO("follome enabled");
      tinker_vision_msgs::EmptyGoalConstPtr goal = enable_as_.acceptNewGoal();
      //return true;
    }
    
    //bool humRecogDisable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    void humRecogDisable()
    {
      enable_ = false;
      init_done_ = false;
      lost_sight_cnt_ = 0;
      start_mat_ = cv::Mat();
      start_mat_masked_ = cv::Mat();
      ROS_INFO("followme disabled");
      tinker_vision_msgs::EmptyGoalConstPtr goal = disable_as_.acceptNewGoal();
      disable_as_.setSucceeded();
      //return true;
    }
    
protected:
    bool is_same_;
    bool enable_;

    cmt::CMT cmt_;
    cv::Rect start_rect_;
    cv::Mat start_mat_;
    cv::Mat start_mat_masked_;
    cv::Mat start_histogram_;
    bool init_done_;
    
    cv::Mat k2_depth_mat_;
    cv::Mat depth_mask_;
    
    //params
    float emd_threshold_;
    float min_depth_;
    float max_depth_;
    int lost_sight_tolerance_;
    int lost_sight_cnt_;
    //const params
    const float resize_p_;
    const float pointcloud_width_tolerance_;
    
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber img_sub_;
    ros::Subscriber k2_body_sub_;
    ros::Subscriber k2_depth_sub_;
    ros::Publisher dbg_pub_;
    ros::Publisher center_pub_;
    //ros::ServiceServer hum_recog_enable_srv_;
    //ros::ServiceServer hum_recog_disable_srv_;
        
    actionlib::SimpleActionServer<tinker_vision_msgs::EmptyAction> enable_as_;
    actionlib::SimpleActionServer<tinker_vision_msgs::EmptyAction> disable_as_;
    
    int abs(int n) {return n>0?n:-n;}
    int min(int a, int b) {return a>b?b:a;}
    int max(int a, int b) {return a<b?b:a;}
    geometry_msgs::Point getZeroPoint() {geometry_msgs::Point p; p.x=0; p.y=0; p.z=0; return p;}
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmt_track_node");
    CMT_track_node n;
    ros::spin();
    return 0;
}


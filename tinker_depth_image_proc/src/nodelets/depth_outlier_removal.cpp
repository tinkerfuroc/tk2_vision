#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tinker_object_recognition/graph_filter/filters.h>


namespace tinker {
namespace vision {

class DepthOutlierRemovalNodelet : public nodelet::Nodelet {
private:
    virtual void onInit() {
        ros::NodeHandle &nh = getNodeHandle();
        // ros::NodeHandle &private_nh = getPrivateNodeHandle();
        it_.reset(new image_transport::ImageTransport(nh));
        image_transport::SubscriberStatusCallback connect_cb = 
            boost::bind(&DepthOutlierRemovalNodelet::ConnectCb, this);
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
        filtered_depth_pub_ =
            it_->advertise("image_filtered", 1, connect_cb, connect_cb);
    }

    void ConnectCb() {
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
        if (filtered_depth_pub_.getNumSubscribers() == 0) {
            raw_depth_sub_.shutdown();
        } else if (!raw_depth_sub_) {
            ROS_INFO("Filter connected!");
            raw_depth_sub_ = it_->subscribe(
                "image_raw", 1, &DepthOutlierRemovalNodelet::DepthCb, this,
                image_transport::TransportHints());
        }
    }

    void DepthCb(const sensor_msgs::ImageConstPtr &raw_depth_msg) {
        ROS_INFO("Filtering depth!");
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(raw_depth_msg);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Check the number of channels
        if (sensor_msgs::image_encodings::numChannels(raw_depth_msg->encoding) != 1) {
            ROS_ERROR("Check channel failed, get %s", raw_depth_msg->encoding.c_str());
            return;
        }

        DilateImage(cv_ptr->image, 5);
        ErodeImage(cv_ptr->image, 5);

        filtered_depth_pub_.publish(cv_ptr->toImageMsg());
    }

    boost::mutex connect_mutex_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher filtered_depth_pub_;
    image_transport::Subscriber raw_depth_sub_;
};

}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tinker::vision::DepthOutlierRemovalNodelet,nodelet::Nodelet);


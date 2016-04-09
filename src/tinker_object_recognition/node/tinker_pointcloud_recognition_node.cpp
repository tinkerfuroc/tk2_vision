#include "tinker_object_recognition/pointcloud_recognition/pointcloud_object_finder.h"

using namespace tinker::vision;

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "tinker_pointcloud_recognition_node");
    ros::NodeHandle private_nh("~/");
    double rate;
    private_nh.param("rate", rate, 2.);
    ros::AsyncSpinner spinner(0);
    PointCloudObjectFinder finder;
    ros::Timer timer = private_nh.createTimer(ros::Duration(1/rate),
            &PointCloudObjectFinder::TimerCallback,
            &finder);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}


#include "ros/ros.h"
#include "kinect_interesting_points/InterestingPoints.h"
#include "pcl_rebuild/Protonect.h"

using namespace tinker::vision;

bool getPoints(kinect_interesting_points::InterestingPoints::Request  &req,
               kinect_interesting_points::InterestingPoints::Response &res)
{
  std::vector<cv::Point3f> points;
  getInterestingPoints(points);
  
  res.cntPoints = points.size();
  for (int i=0; i<res.cntPoints; ++i)
  {
	  res.x.push_back(points[i].x);
	  res.y.push_back(points[i].y);
	  res.z.push_back(points[i].z);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_interesting_points");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("kinect_interesting_points", getPoints);
  ROS_INFO("Ready to return info...");
  ros::spin();

  return 0;
}


#include "ros/ros.h"
#include "kinect_interesting_points/InterestingPoints.h"

bool getPoints(kinect_interesting_points::InterestingPoints::Request  &req,
               kinect_interesting_points::InterestingPoints::Response &res)
{
  res.cntPoints = 12;
  res.x.push_back(1.0f);
  res.y.push_back(-23.0f);
  res.z.push_back(123.0f);
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


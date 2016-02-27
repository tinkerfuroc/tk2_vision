#include "ros/ros.h"
#include "kinect_people_detection/PeoplePosition.h"
#include "pcl_rebuild/people_detection.h"

using namespace tinker::vision;

bool getPoints(kinect_people_detection :: PeoplePosition ::Request  &req,
               kinect_people_detection :: PeoplePosition ::Response &res)
{
  std::vector<cv::Point3f> points;
  getPeoplePosition(points);
  
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
  ros::init(argc, argv, "kinect_people_detection");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("kinect_people_detection", getPoints);
  ROS_INFO("Ready to return info...");
  while(ros::ok())
      ros::spin();
  return 0;
}


#include "ros/ros.h"
#include "kinect_interesting_points/InterestingPoints.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_interesting_points_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<kinect_interesting_points::InterestingPoints>("kinect_interesting_points");
  kinect_interesting_points::InterestingPoints srv;
  if (client.call(srv))
  {
    ROS_INFO("Cnt: %d", (int)srv.response.cntPoints);
  }
  else
  {
    ROS_ERROR("Failed to call service kinect_interesting_points");
    return 1;
  }

  return 0;
}

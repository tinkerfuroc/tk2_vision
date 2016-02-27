#include "ros/ros.h"
#include "kinect_people_detection/PeoplePosition.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_people_detection_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<kinect_people_detection :: PeoplePosition >("kinect_people_detection");
  kinect_people_detection :: PeoplePosition srv;
  if (client.call(srv))
  {
    ROS_INFO("Cnt: %d", (int)srv.response.cntPoints);
	for (int i=0; i<srv.response.cntPoints; ++i)
	{
		ROS_INFO("%d\t%f,%f,%f", i, srv.response.x[i], srv.response.y[i], srv.response.z[i]);
	}
  }
  else
  {
    ROS_ERROR("Failed to call service kinect_people_detection");
    return 1;
  }

  return 0;
}

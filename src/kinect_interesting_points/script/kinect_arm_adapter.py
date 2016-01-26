#!/usr/bin/python

from kinect_interesting_points.srv import InterestingPoints
from geometry_msgs.msg import Point
from std_msgs.msg import String
import rospy

pub = None
now_target_id = 0
kinect_target_x = None
kinect_target_y = None
kinect_target_z = None

def kinect_to_arm(x_kinect, y_kinect, z_kinect):
    arm_point = Point()
    arm_point.x = y_kinect / 100. + 0.38
    arm_point.y = -x_kinect / 100. + 0.35
    arm_point.z = z_kinect / 100. - 0.25
    return arm_point

def arm_arrive_handler():
    now_target_id += 1
    if(now_target_id < len(kinect_target_x)):
        pub.publish(kinect_to_arm(
            kinect_target_x[now_target_id],
            kinect_target_y[now_target_id],
            kinect_target_z[now_target_id]))


if __name__ == '__main__':
    rospy.init_node('kinect_arm_adapter')
    rospy.wait_for_service('kinect_interesting_points')
    pub = rospy.Publisher('arm_target', Point, queue_size=1)
    rospy.Subcriber('arm_arrive', String, arm_arrive_handler)
    get_point_of_interest = rospy.ServiceProxy('kinect_interesting_points', InterestingPoints)
    point_of_interest = get_point_of_interest()
    kinect_target_x = point_of_interest.x
    kinect_target_y = point_of_interest.y
    kinect_target_z = point_of_interest.z
    pub.publish(kinect_to_arm(
        kinect_target_x[now_target_id],
        kinect_target_y[now_target_id],
        kinect_target_z[now_target_id]))
    rospy.spin()



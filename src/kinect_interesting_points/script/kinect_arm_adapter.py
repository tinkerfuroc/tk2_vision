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
    arm_point.x = 0.55
    arm_point.y = -x_kinect / 100. + 0.35
    arm_point.z = y_kinect / 100. + 0.39
    rospy.loginfo('found target:%f %f %f' % (arm_point.x, arm_point.y, arm_point.z))
    return arm_point

def arm_arrive_handler(msg):
    global now_target_id, kinect_target_x, kinect_target_y, kinect_target_z
    print 'arm arrived!'
    while now_target_id < len(kinect_target_x):
        point = kinect_to_arm(
            kinect_target_x[now_target_id],
            kinect_target_y[now_target_id],
            kinect_target_z[now_target_id])
        now_target_id += 1
        if point.z > 0.3 or point.z < -0.1:
            continue
        if point.y > 0.4 or point.y < -0.4:
            continue
        print 'new_target %f %f %f'%(point.x, point.y, point.z)
        print now_target_id
        pub.publish(point)
        break


if __name__ == '__main__':
    rospy.init_node('kinect_arm_adapter')
    rospy.wait_for_service('kinect_interesting_points')
    pub = rospy.Publisher('arm_target', Point, queue_size=1)
    rospy.Subscriber('arm_arrive', String, arm_arrive_handler)
    get_point_of_interest = rospy.ServiceProxy('kinect_interesting_points', InterestingPoints)
    point_of_interest = get_point_of_interest()
    kinect_target_x = point_of_interest.x
    kinect_target_y = point_of_interest.y
    kinect_target_z = point_of_interest.z
    for i in range(0, len(kinect_target_z)):
        kinect_to_arm(kinect_target_x[i], kinect_target_y[i], kinect_target_z[i])
    pub.publish(kinect_to_arm(
        kinect_target_x[now_target_id],
        kinect_target_y[now_target_id],
        kinect_target_z[now_target_id]))
    rospy.spin()



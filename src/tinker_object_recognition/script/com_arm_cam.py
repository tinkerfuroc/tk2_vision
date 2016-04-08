#!/usr/bin/python

import roslib
import sys
import rospy
import numpy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def main(args):
    bridge = CvBridge()
    if len(args) < 2:
        print 'usage: com_arm_webcam camera_id'
        return

    cap = cv2.VideoCapture(int(args[1]))

    image_pub = rospy.Publisher("tk2_com/arm_cam_image", Image, queue_size=100)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            if(cap.isOpened()):
                ret, frame = cap.read()
                image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(e)
            pass
        rate.sleep()
    cap.release()

if __name__ == '__main__':
    rospy.init_node('com_arm_webcam', anonymous=True)
    main(sys.argv)

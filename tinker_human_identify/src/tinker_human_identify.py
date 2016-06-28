#!/usr/bin/python

from oxfordface import find_faces, detect_same_face_in_img, is_same_person
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threading import Lock
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PointStamped
from tinker_vision_msgs.srv import FindOperator, FindOperatorResponse
import numpy as np


def get_bounding_box(face_result):
    from_x = face_result['faceRectangle']['left']
    from_y = face_result['faceRectangle']['top']
    width = face_result['faceRectangle']['width']
    height = face_result['faceRectangle']['height']
    return from_x, from_y, width, height

def face_size(face_result):
    _, _, width, height = get_bounding_box(face_result)
    return width * height

def is_male(face_result):
    return face_result['attributes']['gender'] == 'male'


class TinkerHumanIdentify:
    def __init__(self):
        self._operator_id = None
        rgb_topic = rospy.get_param('~rgb_topic', '/head/kinect2/rgb/image_color')
        loc_topic = rospy.get_param('~loc_topic', '/head/kinect2/depth/image_depth')
        self._rgb_img_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_callback) 
        self._loc_img_sub = rospy.Subscriber(loc_topic, Image, self.depth_callback) 
        self._l = Lock()
        self.bridge = CvBridge()
        self._rgb_img = None
        self._loc_img = None
        self._train_service = rospy.Service('train_operator', Trigger, self.train_operator_callback)
        self._find_service = rospy.Service('find_operator', FindOperator, self.find_operator_callback)
        self._seq = 0
    
    def rgb_callback(self, data):
        with self._l:
            try:
                self._rgb_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)

    def depth_callback(self, data):
        with self._l:
            try:
                self._loc_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            except CvBridgeError as e:
                print(e)

    def train_operator_callback(self, req):
        print 'traing operator'
        result = TriggerResponse()
        result.success = False
        with self._l:
            if self._rgb_img is None:
                return result
            if self.memorize_operator(self._rgb_img):
                result.success = True
                return result
            result.success = False
            return result

    def find_operator_callback(self, req):
        print 'finding operator'
        with self._l:
            if self._rgb_img is None or self._loc_img is None:
                return None
            operator_face = self.reidentify_operator(self._rgb_img)
            if not operator_face:
                return None
            cv2.imwrite('/home/iarc/human_result.png', self._rgb_img)
            from_x, from_y, width, height = get_bounding_box(operator_face)
            to_x = from_x + width
            to_y = from_y + height
            p = PointStamped()
            p.header.stamp = rospy.Time.now()
            p.header.seq = self._seq
            self._seq += 1
            p.header.frame_id = 'kinect_kinect2_rgb_link'
            p.point.x = float(np.mean(self._loc_img[from_y:to_y, from_x:to_x, 2])) / 1000.;
            p.point.y = float(np.mean(self._loc_img[from_y:to_y, from_x:to_x, 0])) / 1000.;
            p.point.z = float(np.mean(self._loc_img[from_y:to_y, from_x:to_x, 1])) / 1000.;
            return p

    def memorize_operator(self, img):
        face_results = find_faces(img)
        if not face_results:
            return False
        face_sizes = [(i, face_size(r)) for i, r in face_results.iteritems()]
        biggest_face_id = max(face_sizes, key=lambda x: x[1])
        self._operator_id = biggest_face_id[0]
        rospy.loginfo('operator id: ' + self._operator_id)
        return True

    def reidentify_operator(self, img):
        found_face_ids, face_results = detect_same_face_in_img(img, self._operator_id)
        self.generate_report_img(img, face_results)
        if not found_face_ids or len(found_face_ids) > 1:
            return None
        operator_face = face_results[found_face_ids[0]]
        from_x, from_y, width, height = get_bounding_box(operator_face)
        from_x += 3
        from_y += 3
        width -= 6
        height -= 6
        to_x = from_x + width
        to_y = from_y + height
        cv2.rectangle(img, (from_x, from_y), (to_x, to_y), (0, 255, 0), 3)
        return operator_face

    def generate_report_img(self, img, found_faces):
        for face_result in found_faces.values():
            from_x, from_y, width, height = get_bounding_box(face_result)
            to_x = from_x + width
            to_y = from_y + height
            if is_male(face_result):
                cv2.rectangle(img, (from_x, from_y), (to_x, to_y), (255, 0, 0), 3)
            else:
                cv2.rectangle(img, (from_x, from_y), (to_x, to_y), (0, 0, 255), 3)

def main():
    rospy.init_node('tinker_human_identify')
    identifier = TinkerHumanIdentify()
    rospy.spin()

if __name__ == '__main__':
    main()

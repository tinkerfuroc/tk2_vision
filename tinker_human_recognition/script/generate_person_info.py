#!/usr/bin/python

from k2_client.msg._BodyArray import BodyArray
from k2_client.msg._JointPositionAndState import JointPositionAndState
from tinker_vision_msgs.msg._TrackPeople import TrackPeople
from tinker_vision_msgs.msg._TrackPerson import TrackPerson
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import rospy
import numpy
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from threading import Lock

def average_position(body):
    sum_x = 0.
    sum_y = 0.
    sum_z = 0.
    for joint in body.jointPositions:
        sum_x += joint.position.x
        sum_y += joint.position.y
        sum_z += joint.position.z
    point = Point(
            sum_x / len(body.jointPositions),
            sum_y / len(body.jointPositions),
            sum_z / len(body.jointPositions)
            )
    

class PersonInfoNode:
    def __init__(self):
        self._people_pub = rospy.Publisher('track_people', TrackPeople, queue_size=10)
        self._image_pub = rospy.Publisher('~/people_track_img', Image, queue_size=10)
        self._new_body = False
        self._new_image = False
        self._body_lock = Lock()
        self._img_lock = Lock()
        self.bridge = CvBridge()
        self.bodies = None
        self.img = None


    def body_handler(self, body_array):
        with self._body_lock:
            self.bodies = body_array.bodies
            self._new_body = True

    def image_handler(self, data):
        with self._img_lock:
            try:
                self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                rospy.logwarn(e)
            self._new_image = True

    def pub_msgs(self):
        with self._body_lock:
            with self._img_lock:
                if not self._new_body or not self._new_image:
                    return
                self._new_body = False
                self._new_image = False
                track_people = TrackPeople()
                for body in self.bodies:
                    person = TrackPerson()
                    person.trackingId = body.trackingId
                    person.pos = average_position(body)
                    person.header = body.header
                    rospy.loginfo("Bounding box")
                    print (body.fromX, body.fromY)
                    print (body.toX, body.toY)
                    track_people.people.append(person)
                    cv2.rectangle(self.img, 
                            (body.fromX, body.fromY), 
                            (body.toX, body.toY),
                            (255, 0, 0), 3)
                self._image_pub.publish(
                        self.bridge.cv2_to_imgmsg(self.img, 'bgr8'))
                self._people_pub.publish(track_people)


def main(argv):
    person_info_node = PersonInfoNode()
    rospy.Subscriber('/head/kinect2/bodyArray', BodyArray, person_info_node.body_handler)
    rospy.Subscriber('/head/kinect2/rgb/image_color', Image, person_info_node.image_handler)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        person_info_node.pub_msgs()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('generate_person_info', anonymous=False)
    main(sys.argv)

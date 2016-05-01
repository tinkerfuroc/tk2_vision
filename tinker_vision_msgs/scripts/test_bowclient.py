#!/usr/bin/env python

import sys
import rospy
from FindObjects.srv import *

def bow_client():
    rospy.wait_for_service('arm_find_objects')
    try:
        find_objects = rospy.ServiceProxy('arm_find_objects', FindObjects)
        response = find_objects()
        print response
        #if (response.success)
        #    return response.objects[0].type.key
        #else
            return "found nothing"
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print "Requesting"
    print "%s"%bow_client()

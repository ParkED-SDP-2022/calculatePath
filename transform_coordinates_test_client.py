#!/usr/bin/env python

import sys
import rospy
from parked_custom_msgs.srv import TransformCoordinates
from parked_custom_msgs.msg import Point

def add_two_ints_client(x, y):
    rospy.wait_for_service('transform_coordinates')
    try:
        add_two_ints = rospy.ServiceProxy('transform_coordinates', TransformCoordinates)
        resp1 = add_two_ints(x, y)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    point = Point()
    point.long = 0.2
    point.lat = 0.123
    tc = TransformCoordinates()
    print(tc)
    print(point)
    print(add_two_ints_client(point, 1))
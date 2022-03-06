#!/usr/bin/env python

import rospy
import actionlib
from parked_custom_msgs.msg import MoveToPointAction, MoveToPointGoal, MoveToPointFeedback, Point

class Navigation_Controller(object):

    _feedback = MoveToPointFeedback
    _goal = MoveToPointGoal

    def __init__(self):

        rospy.init_node('navigation_controller', anonymous=True)
        self._gps_pos = rospy.Subscriber('')
        self._action_server = actionlib.SimpleActionServer('move_to_point', MoveToPointAction, self.handle_move_to_point, auto_start=False)
    
    def handle_move_to_point(self, data):

        

        return 


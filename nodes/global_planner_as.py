#! /usr/bin/env python

from ast import Global
import rospy
import actionlib
from parked_custom_msgs.msg import PlanGlobalPathFeedback, PlanGlobalPathAction, PlanGlobalPathResult, Point

class GlobalPlanner(object):

    _feedback = PlanGlobalPathFeedback()
    _result = PlanGlobalPathResult()


    def __init__(self, name):
        self._action_name = name
        self._gps_pos = None
        self._gps_pos_sub = rospy.Subscriber('bench1/gps_pos', Point, self.update_current_pos)
        self._as = actionlib.SimpleActionServer(self._action_name, PlanGlobalPathAction, execute_cb=self.execute_cb, auto_start=False)
        print('Global Planner Server Starting')
        self._as.start()
        print('Global Planner Server Server started')

    
    def update_current_pos(self, gps_pos):
        self._gps_pos = gps_pos
    
    def execute_cb(self, goal):

        success = True

        goal_pos = goal.destination

        self._as.publish_feedback(self._feedback)

        if success:
            self._as.set_succeeded(self._result)



        
    
if __name__ == '__main__':
    rospy.init_node('bench_x_global_planner')
    server = GlobalPlanner(rospy.get_name())
    rospy.spin()

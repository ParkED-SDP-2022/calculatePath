#!/usr/bin/env python

from threading import local
from matplotlib.transforms import Transform
import rospy
import actionlib
from parked_custom_msgs.msg import MoveToPointAction, MoveToPointGoal, MoveToPointFeedback, MoveToPointResult, NavigateAction, NavigateGoal, Point
from parked_custom_msgs.srv import TransformCoordinates, PlanGlobalPath

class Navigator(object):

    _feedback = MoveToPointFeedback()
    _goal = MoveToPointGoal()
    _result = MoveToPointResult()

    def __init__(self):
        self._current_positon = None
        self._gps_pos = rospy.Subscriber('/robot_position', Point, self.update_current_position, queue_size=5)
        self._action_server = actionlib.SimpleActionServer('move_to_point', MoveToPointAction, self.handle_move_to_point, auto_start=False)
    

    def update_current_position(self, data):
        self._current_positon = data


    def handle_move_to_point(self, goal):
        print(goal)
        self._feedback.message = "goal recieved"
        self._action_server.publish_feedback(self._feedback)

        rospy.wait_for_service('plan_global_path')
        try:
            global_path_service = rospy.ServiceProxy('plan_global_path', PlanGlobalPath)
            global_path = global_path_service(self._current_positon, goal, []).path
        except:
            self._result.long = -999
            self._result.lat = -999
            self._result.angle = -999
            self._action_server.set_aborted(self._result)
            return
        

        rospy.wait_for_service('transform_coordinates')
        try:
            transform_coordinates_service = rospy.ServiceProxy('transform_coordinates', TransformCoordinates)
            transformed_coordinates = transform_coordinates_service(global_path, 1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self._result.long = -999
            self._result.lat = -999
            self._result.angle = -999
            self._action_server.set_aborted(self._result)
            return
        print(transformed_coordinates)
        
        local_planner = actionlib.SimpleActionClient('local_planner', NavigateAction)
        local_planner.wait_for_server()
        print('waiting for lcoal planner server')
        local_planner.send_goal(NavigateGoal(global_path[-1], global_path), done_cb=self.handle_local_planner_finished)
        local_planner.wait_for_result()
        result = local_planner.get_result()
        if result:
            self._result = self._gps_pos
            self._action_server.set_succeeded(self._result)
        return
    
if __name__ == '__main__':
    rospy.init_node('navigation_controller', anonymous=True)
    n = Navigator()
    rospy.spin()

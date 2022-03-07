#! /usr/bin/env python

from ast import Global
from long_lat import LongLat
import rospy
import actionlib
from parked_custom_msgs.msg import Point
from parked_custom_msgs.srv import TransformCoordinates

from Boundaries import Boundaries
from graph import Graph


class GlobalPlanner(object):

    _feedback = PlanGlobalPathFeedback()
    _result = PlanGlobalPathResult()


    def __init__(self, name):
        self._action_name = name
        self._gps_pos = None
        self._gps_pos_sub = rospy.Subscriber('bench1/gps_pos', Point, self.update_current_pos)
        self._front_end_sub = rospy.Subscriber('bench1/navigate', Point, self.execute_frontend_callback)
        print('Global Planner Server Starting')
        self._as.start()
        print('Global Planner Server Server started')
        self.boundary_file = '/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_space_map.geojson' #TODO: pass in file?
        self.boundaries = Boundaries(self.boundary_file)
        self.graph = Graph(self.boundaries)

    def execute_frontend_callback(self, point) {
        # current_pos_pixels = LongLat(self._gps_pos.long self.gps_pos.lat)

        # rospy.wait_for_service('transform_coordinates')
        # try:
        #     tc = rospy.ServiceProxy('transform_coordinates', TransformCoordinates)
        #     resp1 = tc(x, y)
        #     return resp1
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)

        # goal_pos = LongLat(goal.destination.long, goal.destination.lat)
    }

    
    def update_current_pos(self, gps_pos):
        self._gps_pos = gps_pos
    
    def execute_cb(self, goal):

        success = True

        current_pos = LongLat(goal.current_position.long, goal.current_position.lat)
        goal_pos = LongLat(goal.destination.long, goal.destination.lat)
        constraints = self.input_constraints_to_lls(goal.constraints)
        
        
        path = self.graph.GetPath(current_pos, goal_pos, constraints)
        print(path)
        
        self._result.path = self.path_to_point_list(path)

        #self._as.publish_feedback(self._feedback)

        if success:
            self._as.set_succeeded(self._result)

    def input_constraints_to_lls(self, cs):
        result = []
        for c in cs:
            l1 = LongLat(c[0].long, c[0].lat)
            l2 = LongLat(c[1].long, c[1].lat)
            result.append((l1, l2))
        return result
    
    def path_to_point_list(self, path):
        point_list = []
        for nodeA in path:
            point_list.append(Point(nodeA.longLat.long, nodeA.longLat.lat, -999))
        return point_list

        
    
if __name__ == '__main__':
    rospy.init_node('bench_x_global_planner')
    server = GlobalPlanner(rospy.get_name())
    rospy.spin()

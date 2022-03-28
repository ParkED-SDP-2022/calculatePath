#!/usr/bin/env python

# from parked_custom_msgs.srv import PlanGlobalPath,PlanGlobalPathResponse
from parked_custom_msgs.msg import PlanGlobalPathAction, PlanGlobalPathFeedback, PlanGlobalPathResult
from parked_custom_msgs.msg import Point
import rospy
import actionlib

from long_lat import LongLat
from Boundaries import Boundaries
from graph import Graph
import matplotlib.pyplot as plt

class Plan_Global_Path_Server(object):

    _feedback = PlanGlobalPathFeedback()
    _result = PlanGlobalPathResult()


    def __init__(self):
        self._as = actionlib.SimpleActionServer('plan_global_path', PlanGlobalPathAction, execute_cb=self.execute_cb, auto_start=False)
        print('Global Planner Server Starting')
        self._as.start()
        print('Global Planner Server Server started')
        self.boundary_file = '/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_demo_space_from_camera.geojson' #TODO: pass in file?
        self.boundaries = Boundaries(self.boundary_file)
        self.graph = Graph(self.boundaries)

    
    def update_current_pos(self, gps_pos):
        self._gps_pos = gps_pos
    
    def execute_cb(self, goal):
        success = True
        try:
            G = Boundaries('/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_demo_space_from_camera.geojson')

            

            current_pos = LongLat(goal.current_position.long, goal.current_position.lat)
            goal_pos = LongLat(goal.destination.long, goal.destination.lat)
            constraints = self.input_constraints_to_lls(goal.constraints)

            print(current_pos)
            print(goal_pos)
            print(constraints)
            
            path = self.graph.GetPath(current_pos, goal_pos, constraints)
            print(path)
            
            self._result.path = self.path_to_point_list(path)

            #self._as.publish_feedback(self._feedback)

            plot_line_strings = []
            for i in range(len(path) - 1):
                plot_line_strings.append(path[i].longLat.to_LineString(path[i + 1].longLat))
            
            for ls in plot_line_strings:
                x,y = ls.xy
                plt.plot(x,y, color="#ff7040", linewidth=2, solid_capstyle='round')

            G.show_plot()

        except: 
            success = False

        if success:
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(self._result)

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


if __name__ == "__main__":
    rospy.init_node('plan_global_path')
    pgp = Plan_Global_Path_Server()
    while not rospy.is_shutdown():
        rospy.spin()





# class Plan_Global_Path_Server(object):


#     def __init__(self):
#         self.boundary_file = '/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_space_map.geojson' #TODO: pass in file?
#         # self.boundary_file = '/home/arehman/catkin_ws/src/calculatePath/sdp_demo_space_from_camera.geojson' #TODO: pass in file?
#         print('file name set')
#         self.boundaries = Boundaries(self.boundary_file)
#         print('boundaries set')
#         self.graph = Graph(self.boundaries)

#         print('starting plan_global_path server')
#         s = rospy.Service('plan_global_path', PlanGlobalPath, self.handle_plan_global_path)

#         print('plan_global_path server started')

    
#     def handle_plan_global_path(self, data):
#         current_position = LongLat(data.current_position.long, data.current_position.lat)
#         goal_
#     def input_constraints_to_lls(self, cs):
#         result = []
#         for c in cs:
#             l1 = LongLat(c[0].long, c[0].lat)
#             l2 = LongLat(c[1].long, c[1].lat)
#             result.append((l1, l2))
#         return result

    
#     def path_to_point_list(self, path):
#         point_list = []
#         for nodeA in path:
#             point_list.append(Point(nodeA.longLat.long, nodeA.longLat.lat, -999))
#         return point_list
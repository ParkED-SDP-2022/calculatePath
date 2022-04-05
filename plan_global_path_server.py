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
        self.boundary_file = '/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_demo_space_from_camera_2.geojson' #TODO: pass in file?
        self.boundaries = Boundaries(self.boundary_file)
        # self.graph = Graph(self.boundaries)

    
    def update_current_pos(self, gps_pos):
        self._gps_pos = gps_pos
    
    def execute_cb(self, goal):
        success = True
        try:
            G = Boundaries('/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_demo_space_from_camera_2.geojson')
            graph = Graph(G)

            
            ##print("goal: ", goal.current_position.long, " current_pos: ", goal.current_po )
            #print("goaalll")
            #print(goal)

            current_pos = LongLat(goal.current_position.long, goal.current_position.lat)
            goal_pos = LongLat(goal.destination.long, goal.destination.lat)
            #constraints = self.input_constraints_to_lls(goal.constraints)
            if (len(goal.constraints) % 2 != 0):
                print("WARNING CONSTRAINTS MUST BE PASSED IN GROUPS OF TWO. They represent an edge, not a node :^)")

            cons_list_of_Point = goal.constraints
            constraint = self.points_to_edges(cons_list_of_Point)

            print("\nconstraints ::")
            for c in constraint:
                print(str(c[0]), str(c[1]))
            
            path = graph.GetPath(current_pos, goal_pos, constraint)

            ##############################################

            # G = Boundaries('/afs/inf.ed.ac.uk/user/s18/s1829279/Desktop/sdp/catkin_ws/src/calculatePath/sdp_demo_space_from_camera.geojson')
            # graph = Graph(G)

            

            # current_pos = LongLat(goal.current_position.long, goal.current_position.lat)
            # current_pos = path[2]
            # goal_pos = LongLat(goal.destination.long, goal.destination.lat)
            # constraints = self.input_constraints_to_lls(goal.constraints)
            # constraints.append(path[3])
            # path = graph.GetPath(current_pos, goal_pos, constraints)

            ##############################################
            print("====================== actual path ==========================")
            for node in path:
                print(node.longLat)
            
            self._result.path = self.path_to_point_list(path)

            #self._as.publish_feedback(self._feedback)

            plot_line_strings = []
            for i in range(len(path) - 1):
                plot_line_strings.append(path[i].longLat.to_LineString(path[i + 1].longLat))
            i = 0
            for ls in plot_line_strings:
                x,y = ls.xy
                if i == 0:
                    plt.plot(x,y, color="#00FF00", linewidth=2, solid_capstyle='round')
                else:
                    plt.plot(x,y, color="#ff7040", linewidth=2, solid_capstyle='round')
                i += 1

            if constraint:
                for pair in constraint:
                    midpoint = pair[0].midpoint(pair[1])
                    plt.scatter(midpoint.long, midpoint.lat, marker='H', color='red', s=300)

            G.show_plot()
            plt.clf()

        except Exception as ex: 
            print(ex)
            success = False

        if success:
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(self._result)

    def point_to_long_lat(self, point):
        return LongLat(point.long, point.lat)

    ## [Point] -> [tuple(LongLat)]
    def points_to_edges(self, points):
        edges = []
        if len(points) == 0:
            return edges
        for i in range(len(points)-1):
            node1 = self.point_to_long_lat(points[i])
            node2 = self.point_to_long_lat(points[i+1])
            edges.append((node1, node2))
        return edges




    # def input_constraints_to_lls(self, cs):
    #     result = []

    #     #l1 = LongLat(cs[0].long,cs[0].lat)
    #     #l2 = LongLat(cs[1].long,cs[1].lat)

    #     #return [(l1,l2)]
    #     for c in cs:
    #         l1 = LongLat(c.long, c.lat)
    #         l2 = LongLat(c.long, c.lat)
    #         result.append((l1, l2))
    #     return result
    
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
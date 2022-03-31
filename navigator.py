#!/usr/bin/env python

from threading import local
from matplotlib.transforms import Transform
import rospy
import actionlib
from parked_custom_msgs.msg import MoveToPointAction, MoveToPointGoal, MoveToPointFeedback, MoveToPointResult, NavigateAction, NavigateGoal, PlanGlobalPathAction, PlanGlobalPathGoal, Point
from parked_custom_msgs.srv import TransformCoordinates

class Navigator(object):

    _feedback = MoveToPointFeedback()
    _goal = MoveToPointGoal()
    _result = MoveToPointResult()

    def __init__(self):
        self.LAT_MIN = 0.0
        self.LAT_MAX = 1.2631578947
        self.LONG_MIN = 0.0
        self.LONG_MAX = 1.0
        self.IMAGE_X = 1200
        self.IMAGE_Y = 950
        self._current_positon = Point(0.5,0.5,-999)
        self._gps_pos = rospy.Subscriber('/robot_position_longlat', Point, self.update_current_position, queue_size=5)
        self._action_server = actionlib.SimpleActionServer('move_to_point', MoveToPointAction, self.handle_move_to_point, auto_start=False)
        self._action_server.start()
        print('Navigator server has been started')
    

    def update_current_position(self, data):
        self._current_positon = data


    def handle_move_to_point(self, goal):
        print(goal)
        self._feedback.message = "goal recieved"
        self._action_server.publish_feedback(self._feedback)

        gp_goal = PlanGlobalPathGoal()
        # gp_goal.current_position = self._current_positon
        gp_goal.current_position = Point()
        gp_goal.current_position.long = self._current_positon.long
        gp_goal.current_position.lat = self._current_positon.lat
        gp_goal.destination = goal.destination
        gp_goal.constraints = []

        global_planner_ac = actionlib.SimpleActionClient('plan_global_path', PlanGlobalPathAction)
        global_planner_ac.wait_for_server()
        global_planner_ac.send_goal(gp_goal)
        global_planner_ac.wait_for_result()
        global_path = global_planner_ac.get_result().path
        

        # rospy.wait_for_service('transform_coordinates')
        try:
            # transform_coordinates_service = rospy.ServiceProxy('transform_coordinates', TransformCoordinates)
            # transformed_coordinates = transform_coordinates_service(global_path, 1)
            transformed_coordinates = self.transform_coordinates(global_path, 1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            # self._result.long = -999
            # self._result.lat = -999
            # self._result.angle = -999
            self._action_server.set_aborted(self._result)
            return
        print(transformed_coordinates)


        transformed_destination = self.transform_coordinates([goal.destination], 1)[0]
        
        local_planner = actionlib.SimpleActionClient('bench_x_local_planner', NavigateAction)
        local_planner.wait_for_server()
        print('waiting for lcoal planner server')

        navigation_goal = NavigateGoal()
        navigation_goal.destination = transformed_destination
        navigation_goal.path = transformed_coordinates

        local_planner.send_goal(navigation_goal)
        local_planner.wait_for_result()
        result = local_planner.get_result()
        if result:
            self._result = self._gps_pos
            self._action_server.set_succeeded(self._result)
        return

    def transform_coordinates(self, data, flag):
        
        print(data)
        change_in_Long = self.LONG_MAX - self.LONG_MIN
        change_in_lat = self.LAT_MAX - self.LAT_MIN
        image_x = self.IMAGE_X
        image_y = self.IMAGE_Y

        processed_points = []
        for point in data:
            long_conversion_constant = image_y / change_in_Long
            lat_conversion_constant = image_x / change_in_lat

            point_to_convert = point

            if flag == 1:
                point_to_convert.long = long_conversion_constant * point_to_convert.long
                point_to_convert.lat = lat_conversion_constant * point_to_convert.lat
            if flag == 2:
                point_to_convert.long = (1 / long_conversion_constant) * point_to_convert.long
                point_to_convert.lat = (1 / lat_conversion_constant) * point_to_convert.lat

            processed_points.append(point_to_convert)

        return processed_points
    
if __name__ == '__main__':
    rospy.init_node('navigator', anonymous=True)
    n = Navigator()
    while not rospy.is_shutdown():
        rospy.spin()

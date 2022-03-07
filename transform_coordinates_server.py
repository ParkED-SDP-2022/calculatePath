#!/usr/bin/env python

from calendar import prcal
from parked_custom_msgs.srv import TransformCoordinates,TransformCoordinatesResponse
import rospy

class Transform_Coordinates_server(object):

    def __init__(self):
        self.LONG_MIN = 0.0
        self.LONG_MAX = 1.2631578947
        self.LAT_MIN = 0.0
        self.LAT_MAX = 1.0
        self.IMAGE_X = 1200
        self.IMAGE_Y = 950


        rospy.init_node('transform_coordinates_server')
        s = rospy.Service('/transform_coordinates', TransformCoordinates, self.handle_transform_coordinates)
        print('Transform Coordinates Server started')
    
    def handle_transform_coordinates(self, data):
        
        print(data)
        change_in_Long = self.LONG_MAX - self.LONG_MIN
        change_in_lat = self.LAT_MAX - self.LAT_MIN
        image_x = self.IMAGE_X
        image_y = self.IMAGE_Y

        processed_points = []
        for point in data.inputPositions:
            long_conversion_constant = image_y / change_in_Long
            lat_conversion_constant = image_x / change_in_lat

            point_to_convert = point

            if data.flag == 1:
                point_to_convert.long = long_conversion_constant * point_to_convert.long
                point_to_convert.lat = lat_conversion_constant * point_to_convert.lat
            if data.flag == 2:
                point_to_convert.long = (1 / long_conversion_constant) * point_to_convert.long
                point_to_convert.lat = (1 / lat_conversion_constant) * point_to_convert.lat

            processed_points.append(point_to_convert)

        return TransformCoordinatesResponse(processed_points)


if __name__ == "__main__":
    tcv = Transform_Coordinates_server()
    rospy.spin()


#!/usr/bin/env python
import rospy
from parked_custom_msgs.msg import Point
from std_msgs.msg import String, Bool
from heat_map import HeatMapList
import datetime

class HeatNode:
    def __init__(self):
        self.heat_map = HeatMapList()
        # subscribes to pressure sensor data. The payload of the the message will just be a boolean
        self.pressure_subscriber = rospy.Subscriber('/pressure_data', Bool,
                                                    self.pressure_subscriber_cb)

        # subscribes to robot's current position. The payload of the message will be a parked_custom_msgs/Point object
        self.position_subscriber = rospy.Subscriber('/robot_positon', Point, self.update_current_position)
        self.current_position = None

        # publishes string notation of geojson
        self.heat_map_publisher = rospy.Publisher('/heat_map_data', String, queue_size=5)
        self.publisher_msg = String()

        rospy.init_node('heat_node', anonymous=True)
        rospy.spin()


    def pressure_subscriber_cb(self, pressure_data):
        if self.current_position == None:
            return
        
        longitude = self.current_position.long
        latitude = self.current_position.lat
        sit_down = pressure_data.sit_down.data
        time = datetime.datetime.now()
        long_lat = (longitude, latitude)

        self.heat_map.change_state(long_lat, time, sit_down)

        if not sit_down:
            geojson = self.heat_map.to_geojson().dump()
            self.publisher_msg.data = geojson
            self.heat_map_publisher.publish(self.publisher_msg)
    
    def update_current_position(self, data):
        self.current_position = data
    
if __name__ == '__main__':
    hn = HeatNode()






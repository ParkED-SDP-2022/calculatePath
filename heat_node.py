#!/usr/bin/env python
import rospy
from parked_custom_msgs.msg import Point
from std_msgs.msg import String, Bool
from heat_map import HeatMapList # todo we need to change message type to handle bench_id
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

        # todo we need to use this bench_id in the calling of these methods
        self.heat_map.change_state(long_lat, time, sit_down)

        if not sit_down:
            geojson = self.heat_map.to_geojson_heatmap().dump()
            self.publisher_msg.data = geojson
            self.heat_map_publisher.publish(self.publisher_msg)

        # todo we need to make use of the to_geojson_benchstate() method publishing its fruits to frontend
    
    def update_current_position(self, data):
        self.current_position = data
    
if __name__ == '__main__':
    hn = HeatNode()






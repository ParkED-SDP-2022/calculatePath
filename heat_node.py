import rospy
from parked_custom_msgs import Pressure_Sensor
from std.msgs.msg import String
from heat_map import HeatMapList
import datetime

class HeatNode:
    def __init__(self):
        self.heat_map = HeatMapList()
        self.pressure_subscriber = rospy.Subscriber('pressure_data', Pressure_Sensor,
                                                    self.pressure_subscriber_cb)
        self.position_subscriber = rospy.Subscriber
        self.heat_map_publisher = rospy.Publisher('heat_map_data', String, queue_size=5)
        self.publisher_msg = String()
        rospy.init_node('HeatNode', anonymous=True)
        rospy.spin()


    def pressure_subscriber_cb(self, pressure_data):
        longitude = pressure_data.longitude.data
        latitude = pressure_data.latitude.data
        sit_down = pressure_data.sit_down.data
        time = datetime.datetime.now()
        long_lat = (longitude, latitude)
        self.heat_map.change_state(long_lat, time, sit_down)
        if not sit_down:
            geojson = self.heat_map.to_geojson().dump()
            self.publisher_msg.data = geojson
            self.heat_map_publisher.publish(self.publisher_msg)







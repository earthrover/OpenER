import math
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, Quaternion, Point

from sensor_msgs.msg import NavSatFix
from earth_rover_navigation.srv import *
import earth_rover_navigation.srv as srv

api = None

class NavigationApi(object):

    def __init__(self):

        self.start_service = rospy.ServiceProxy('/earth_rover_navigation/start_navigation', srv.start_navigation)
        self.pause_service = rospy.ServiceProxy('/earth_rover_navigation/pause_navigation', srv.pause_navigation)
        self.cancel_service = rospy.ServiceProxy('/earth_rover_navigation/cancel_navigation', srv.cancel_navigation)
        self.add_waypoint_service = rospy.ServiceProxy('/earth_rover_navigation/add_geo_waypoint', srv.geo_wp_srv)
        self.add_waypoint_service_c = rospy.ServiceProxy('/earth_rover_navigation/add_cartesian_waypoint', srv.cartesian_wp_srv)


def init_api():
    global api
    api = NavigationApi()

def start():
    api.start_service()

def pause():
    api.pause_service()

def cancel():
    api.cancel_service()

def add_waypoint_cartesian(x, y, bearing):
    position = Point(x=x, y=y)
    east = math.sin(math.radians(bearing))
    north = math.cos(math.radians(bearing))
    orientation = Quaternion(x=east, y=north, z=0.0, w=1.0)
    pose = Pose(position=position, orientation=orientation)
    api.add_waypoint_service_c(cartesian_wp=[pose])


def add_waypoint(lat, lon, bearing):
    fix = NavSatFix(latitude=lat, longitude=lon)
    east = math.sin(math.radians(bearing))
    north = math.cos(math.radians(bearing))
    orientation = Quaternion(x=east, y=north, z=0.0, w=1.0)
    api.add_waypoint_service(gps_wp=[fix], orientation=[orientation])


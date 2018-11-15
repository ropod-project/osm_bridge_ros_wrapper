#!/usr/bin/env python
import overpass
import utm
import rospy
import actionlib
from osm_bridge_ros_wrapper.msg import OSMQueryAction, OSMQueryGoal, OSMQueryResult 
from osm_bridge_ros_wrapper.msg import WMQueryAction, WMQueryGoal, WMQueryResult 
from osm_bridge_ros_wrapper.msg import PathPlannerAction, PathPlannerGoal, PathPlannerResult 
from osm_bridge_ros_wrapper.msg import GridMapGeneratorAction, GridMapGeneratorGoal, GridMapGeneratorResult 

class OSMBridge(object):

  def __init__(self):
      server_ip = rospy.get_param('~overpass_server_ip')
      server_port = rospy.get_param('~overpass_server_port')
      ref_lat = rospy.get_param('~ref_latitude')
      ref_lon = rospy.get_param('~ref_longitude')

      self.wm_query_server = actionlib.SimpleActionServer('/wm_query', WMQueryAction, self.wm_query, False)
      self.wm_query_server.start()

      self.osm_query_server = actionlib.SimpleActionServer('/wm_query', OSMQueryAction, self.osm_query, False)
      self.osm_query_server.start()

      self.path_planner_server = actionlib.SimpleActionServer('/path_planner', PathPlannerAction, self.path_planner, False)
      self.path_planner_server.start()

      self.grid_map_generator_server = actionlib.SimpleActionServer('/grid_map_generator', GridMapGeneratorAction, self.grid_map_generator, False)
      self.grid_map_generator_server.start()

  def wm_query(self, req):
      res = WMQueryResult()
      '''

      '''
      self.wm_query_server.set_succeeded(res)

  def osm_query(self, req):
      res = OSMQueryResult()
      '''

      '''
      self.osm_query_server.set_succeeded(res)

  def path_planner(self, req):
      res = PathPlannerResult()
      '''

      '''
      self.path_planner_server.set_succeeded(res)

  def grid_map_generator(self, req):
      res = GridMapGeneratorResult()
      '''

      '''
      self.grid_map_generator_server.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node('OSMBridge')
    osm_bridge = OSMBridge()
    
    rospy.spin()

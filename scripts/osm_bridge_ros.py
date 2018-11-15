#!/usr/bin/env python

PACKAGE = 'osm_bridge_ros_wrapper'
NODE = 'osm_bridge_ros'

# import overpass
# import utm
from OBL import OSMBridge, OSMAdapter, OccGridGenerator
import rospy
from actionlib import SimpleActionServer 
from osm_bridge_ros_wrapper.msg import *
from osm_query_callback import OSMQueryCallback
# from osm_bridge_ros_wrapper.msg import OSMQueryAction, OSMQueryGoal, OSMQueryResult 
# from osm_bridge_ros_wrapper.msg import WMQueryAction, WMQueryGoal, WMQueryResult 
# from osm_bridge_ros_wrapper.msg import PathPlannerAction, PathPlannerGoal, PathPlannerResult 
# from osm_bridge_ros_wrapper.msg import GridMapGeneratorAction, GridMapGeneratorGoal, GridMapGeneratorResult 

class OSMBridgeROS(object):

    def __init__(self):
        server_ip = rospy.get_param('~overpass_server_ip')
        server_port = rospy.get_param('~overpass_server_port')
        ref_lat = rospy.get_param('~ref_latitude')
        ref_lon = rospy.get_param('~ref_longitude')
        global_origin = [ref_lat, ref_lon]

        rospy.loginfo("Server " + server_ip + ":" + str(server_port))
        rospy.loginfo("Global origin: " + str(global_origin))
        rospy.loginfo("Starting servers...")

        self.osm_bridge = OSMBridge(
                server_ip=server_ip,
                server_port=server_port,
                global_origin=global_origin,
                coordinate_system="cartesian",
                debug=False)
        self.osm_adapter = OSMAdapter(
                server_ip=server_ip,
                server_port=server_port,
                debug=False)
        self.occ_grid_generator = OccGridGenerator(
                server_ip=server_ip,
                server_port=server_port,
                global_origin=global_origin,
                debug=False)

        self.wm_query_server = SimpleActionServer('/wm_query', WMQueryAction, self._wm_query, False)
        self.wm_query_server.start()

        self.osm_query_server = SimpleActionServer('/osm_query', OSMQueryAction, self._osm_query, False)
        self.osm_query_server.start()
        self.osm_query_callback = OSMQueryCallback(self.osm_adapter)

        self.path_planner_server = SimpleActionServer('/path_planner', PathPlannerAction, self._path_planner, False)
        self.path_planner_server.start()

        self.grid_map_generator_server = SimpleActionServer('/grid_map_generator', GridMapGeneratorAction, self._grid_map_generator, False)
        self.grid_map_generator_server.start()

        rospy.loginfo("Servers started. Listening for queries...")

    def _wm_query(self, req):
        res = WMQueryResult()
        '''

        '''
        self.wm_query_server.set_succeeded(res)

    def _osm_query(self, req):
        ''' Access the input of the osm_query message and ask OSMBridge for it.
        After receiving info from OSMBridge, it packages it as response message and send it back.
        '''
#         rospy.loginfo(req)
        res = self.osm_query_callback.get_response(req)
        self.osm_query_server.set_succeeded(res)

    def _path_planner(self, req):
        res = PathPlannerResult()
        '''

        '''
        self.path_planner_server.set_succeeded(res)

    def _grid_map_generator(self, req):
        '''

        '''
        res = GridMapGeneratorResult()
        self.grid_map_generator_server.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node(NODE)
    osm_bridge_ros = OSMBridgeROS()
    rospy.spin()

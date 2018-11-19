#!/usr/bin/env python

PACKAGE = 'osm_bridge_ros_wrapper'
NODE = 'osm_bridge_ros'

from OBL import OSMBridge, OSMAdapter, OccGridGenerator, PathPlanner
import rospy
from actionlib import SimpleActionServer 
from osm_bridge_ros_wrapper.msg import *
from osm_query_callback import OSMQueryCallback
from wm_query_callback import WMQueryCallback
from obl_wm_to_ros_adapter import OBLWMToROSAdapter

class OSMBridgeROS(object):

    def __init__(self):
        server_ip = rospy.get_param('~overpass_server_ip')
        server_port = rospy.get_param('~overpass_server_port')
        ref_lat = rospy.get_param('~ref_latitude')
        ref_lon = rospy.get_param('~ref_longitude')
        building = rospy.get_param('~building')
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
        self.wm_query_callback = WMQueryCallback(self.osm_bridge)

        self.osm_query_server = SimpleActionServer('/osm_query', OSMQueryAction, self._osm_query, False)
        self.osm_query_server.start()
        self.osm_query_callback = OSMQueryCallback(self.osm_adapter)

        self.path_planner_server = SimpleActionServer('/path_planner', PathPlannerAction, self._path_planner, False)
        self.path_planner = PathPlanner(self.osm_bridge)
        self.path_planner.set_building(building)
        self.path_planner_server.start()

        self.grid_map_generator_server = SimpleActionServer('/grid_map_generator', GridMapGeneratorAction, self._grid_map_generator, False)
        self.grid_map_generator_server.start()

        rospy.loginfo("Servers started. Listening for queries...")

    def _wm_query(self, req):
        ''' Access the fields of goal of the wm_query message and ask OSMBridge for it.
        After receiving info from OSMBridge, it packages it as response message and send it back.
        '''
        res = self.wm_query_callback.get_response(req)
        if res is not None:
            self.wm_query_server.set_succeeded(res)
        else:
            self.wm_query_server.set_rejected(res)

    def _osm_query(self, req):
        ''' Access the fields of goal of the osm_query message and ask OSMBridge for it.
        After receiving info from OSMBridge, it packages it as response message and send it back.
        '''
#         rospy.loginfo(req)
        res = self.osm_query_callback.get_response(req)
        self.osm_query_server.set_succeeded(res)

    def _path_planner(self, req):
        '''Access the fields of goal of PathPlanner.action message and call
        OSM Bridge Path planner with those parameters and return the response to the sender.
        '''
        start_floor = req.start_floor
        destination_floor = req.destination_floor
        start_area = req.start_area
        destination_area = req.destination_area
        start_local_area = req.start_local_area
        destination_local_area = req.destination_local_area
        start_position = req.start_position
        destination_task = req.destination_task

        path = []

        if start_local_area and destination_local_area:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor,start_area=start_area,destination_area=destination_area, start_local_area=start_local_area,destination_local_area=destination_local_area)
        elif start_local_area and destination_task:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor,start_area=start_area,destination_area=destination_area, start_local_area=start_local_area,destination_task=destination_task)
        elif start_position and destination_task:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor,start_area=start_area,destination_area=destination_area, robot_position=start_position,destination_task=destination_task)
        elif start_position and destination_local_area:
            path = self.path_planner.get_path_plan(start_floor=start_floor, destination_floor=destination_floor,start_area=start_area,destination_area=destination_area, robot_position=start_position,destination_local_area=destination_local_area)
        else:
            rospy.logwarn("Path planner need more arguments to plan the path")

        
        res = PathPlannerResult()
        for pt in path:
            res.planner_areas.append(OBLWMToROSAdapter.get_planner_area(pt))
        
        self.path_planner_server.set_succeeded(res)

    def _grid_map_generator(self, req):
        '''Access the fields of goal of GridMapGenerator.action message and call
        OccGridGenerator with those parameters and return the response to the sender.
        '''
        local_offset_x = req.local_offset_x
        local_offset_y = req.local_offset_y
        resolution = req.resolution
        dimension = req.dimension
        dirname = req.dirname
        filename = req.filename
        floor = req.floor
        self.occ_grid_generator.setDimension(dimension)
        self.occ_grid_generator.setResolution(resolution)
        self.occ_grid_generator.setDirName(dirname)
        self.occ_grid_generator.setFileName(filename)
        self.occ_grid_generator.setLocalOffset([local_offset_x, local_offset_y])
        filename = self.occ_grid_generator.generate_map(floor=floor)
        res = GridMapGeneratorResult()
        res.filename = filename
        self.grid_map_generator_server.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node(NODE)
    osm_bridge_ros = OSMBridgeROS()
    rospy.spin()

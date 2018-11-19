from osm_bridge_ros_wrapper.msg import *
from obl_wm_to_ros_adapter import OBLWMToROSAdapter
import rospy

class WMQueryCallback(object):

    """callback for wm query server"""

    def __init__(self, osm_bridge):
        self.osm_bridge = osm_bridge

    def get_response(self, req):
        ref = req.id or req.ref
        entity_type = req.type
        scope_id = req.scope_id
        scope_type = req.scope_type
        
        res = WMQueryResult()
        res.output = None

        try:
            if entity_type == 'building':
                res.output = 'building'
                building_obj = self.osm_bridge.get_building(ref)
                res.building = OBLWMToROSAdapter.get_building_msg_from_building_obj(building_obj)
            elif entity_type == 'floor':
                res.output = 'floor'
                floor_obj = self.osm_bridge.get_floor(ref)
                res.floor = OBLWMToROSAdapter.get_floor_msg_from_floor_obj(floor_obj)
            elif entity_type == 'elevator':
                res.output = 'elevator'
                elevator_obj = self.osm_bridge.get_elevator(ref)
                res.elevator = OBLWMToROSAdapter.get_elevator_msg_from_elevator_obj(elevator_obj)
            elif entity_type == 'stairs':
                res.output = 'stairs'
                stairs_obj = self.osm_bridge.get_stairs(ref)
                res.stairs = OBLWMToROSAdapter.get_stairs_msg_from_stairs_obj(stairs_obj)
            elif entity_type == 'room':
                res.output = 'room'
                room_obj = self.osm_bridge.get_room(ref)
                res.room = OBLWMToROSAdapter.get_room_msg_from_room_obj(room_obj)
            elif entity_type == 'area':
                res.output = 'area'
                area_obj = self.osm_bridge.get_area(ref)
                res.area = OBLWMToROSAdapter.get_area_msg_from_room_obj(room_obj)
            elif entity_type == 'corridor':
                res.output = 'corridor'
                corridor_obj = self.osm_bridge.get_corridor(ref)
                res.corridor = OBLWMToROSAdapter.get_corridor_msg_from_corridor_obj(corridor_obj)
            elif entity_type == 'connection':
                res.output = 'connection'
                connection_obj = self.osm_bridge.get_connection(ref)
                res.connection = OBLWMToROSAdapter.get_connection_msg_from_connection_obj(connection_obj)
            elif entity_type == 'local_area':
                res.output = 'local_area'
                local_area_obj = self.osm_bridge.get_local_area(ref)
                local_area_obj.geometry
                res.local_area = OBLWMToROSAdapter.get_local_area_msg_from_local_area_obj(local_area_obj)
            elif entity_type == 'door':
                res.output = 'door'
                door_obj = self.osm_bridge.get_door(ref)
                res.door = OBLWMToROSAdapter.get_door_msg_from_door_obj(door_obj)
            elif entity_type == 'wall':
                res.output = 'wall'
                wall_obj = self.osm_bridge.get_wall(ref)
                res.wall = OBLWMToROSAdapter.get_wall_msg_from_wall_obj(wall_obj)
            elif entity_type == 'side':
                res.output = 'side'
                side_obj = self.osm_bridge.get_side(ref)
                res.side = OBLWMToROSAdapter.get_side_msg_from_side_obj(side_obj)
            elif entity_type == 'feature':
                res.output = 'feature'
                feature_obj = self.osm_bridge.get_feature(ref)
                res.feature = OBLWMToROSAdapter.get_feature_msg_from_feature_obj(feature_obj)
            elif entity_type == 'point':
                res.output = 'point'
                point_obj = self.osm_bridge.get_point(ref)
                print(point_obj)
                res.point = OBLWMToROSAdapter.get_point_msg_from_point_obj(point_obj)
            elif entity_type == 'shape':
                res.output = 'shape'
                shape_obj = self.osm_bridge.get_shape(ref)
                res.shape = OBLWMToROSAdapter.get_shape_msg_from_shape_obj(shape_obj)

            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    
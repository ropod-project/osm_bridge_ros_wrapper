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

        res = WMQueryResult()
        res.output = None

        try:
            if entity_type == 'building':
                res.output = 'building'
                building_obj = self.osm_bridge.get_building(ref)
                res.building = OBLWMToROSAdapter.get_building_msg_from_building_obj(building_obj)
            elif entity_type == 'floor':
                res.output = 'floor'
                if scope_id == 0:
                    floor_obj = self.osm_bridge.get_floor(ref)
                else:
                    floor_obj = self.osm_bridge.get_floor(ref,scope_id=scope_id, scope_role='floor',scope_role_type='relation')

                res.floor = OBLWMToROSAdapter.get_floor_msg_from_floor_obj(floor_obj)
            elif entity_type == 'elevator':
                res.output = 'elevator'
                if scope_id == 0:
                    elevator_obj = self.osm_bridge.get_elevator(ref)
                else:
                    elevator_obj = self.osm_bridge.get_elevator(ref,scope_id=scope_id, scope_role='elevator',scope_role_type='relation')
                res.elevator = OBLWMToROSAdapter.get_elevator_msg_from_elevator_obj(elevator_obj)
            elif entity_type == 'stairs':
                res.output = 'stairs'
                if scope_id == 0:
                    stairs_obj = self.osm_bridge.get_stairs(ref)
                else:
                    stairs_obj = self.osm_bridge.get_stairs(ref,scope_id=scope_id, scope_role='stairs',scope_role_type='relation')
                res.stairs = OBLWMToROSAdapter.get_stairs_msg_from_stairs_obj(stairs_obj)
            elif entity_type == 'room':
                res.output = 'room'
                if scope_id == 0:
                    room_obj = self.osm_bridge.get_room(ref)
                else:
                    room_obj = self.osm_bridge.get_room(ref,scope_id=scope_id, scope_role='room',scope_role_type='relation')
                res.room = OBLWMToROSAdapter.get_room_msg_from_room_obj(room_obj)
            elif entity_type == 'area':
                res.output = 'area'
                if scope_id == 0:
                    area_obj = self.osm_bridge.get_area(ref)
                else:
                    area_obj = self.osm_bridge.get_area(ref,scope_id=scope_id, scope_role='area', scope_role_type='relation')
                res.area = OBLWMToROSAdapter.get_area_msg_from_room_obj(room_obj)
            elif entity_type == 'corridor':
                res.output = 'corridor'
                if scope_id == 0:
                    corridor_obj = self.osm_bridge.get_corridor(ref)
                else:
                    corridor_obj = self.osm_bridge.get_corridor(ref,scope_id=scope_id, scope_role='area', scope_role_type='relation')
                res.corridor = OBLWMToROSAdapter.get_corridor_msg_from_corridor_obj(corridor_obj)
            elif entity_type == 'connection':
                res.output = 'connection'
                connection_obj = self.osm_bridge.get_connection(ref)
                res.connection = OBLWMToROSAdapter.get_connection_msg_from_connection_obj(connection_obj)
            elif entity_type == 'local_area':
                res.output = 'local_area'
                if scope_id == 0:
                    local_area_obj = self.osm_bridge.get_local_area(ref)
                else:
                    local_area_obj = self.osm_bridge.get_local_area(ref,scope_id=scope_id, scope_role='local_area', scope_role_type='relation')
                local_area_obj.geometry
                res.local_area = OBLWMToROSAdapter.get_local_area_msg_from_local_area_obj(local_area_obj)
            elif entity_type == 'door':
                res.output = 'door'
                if scope_id == 0:
                    door_obj = self.osm_bridge.get_door(ref)
                else:
                    door_obj = self.osm_bridge.get_door(ref,scope_id=scope_id, scope_role='door', scope_role_type='relation')
                res.door = OBLWMToROSAdapter.get_door_msg_from_door_obj(door_obj)
            elif entity_type == 'wall':
                res.output = 'wall'
                if scope_id == 0:
                    wall_obj = self.osm_bridge.get_wall(ref)
                else:
                    wall_obj = self.osm_bridge.get_wall(ref,scope_id=scope_id, scope_role='wall', scope_role_type='relation')
                res.wall = OBLWMToROSAdapter.get_wall_msg_from_wall_obj(wall_obj)
            elif entity_type == 'side':
                res.output = 'side'
                if scope_id == 0:
                    side_obj = self.osm_bridge.get_side(ref)
                else:
                    side_obj = self.osm_bridge.get_side(ref,scope_id=scope_id, scope_role='side', scope_role_type='relation')
                res.side = OBLWMToROSAdapter.get_side_msg_from_side_obj(side_obj)
            elif entity_type == 'feature':
                res.output = 'feature'
                if scope_id == 0:
                    feature_obj = self.osm_bridge.get_feature(ref)
                else:
                    feature_obj = self.osm_bridge.get_feature(ref,scope_id=scope_id, scope_role='side', scope_role_type='relation')
                res.feature = OBLWMToROSAdapter.get_feature_msg_from_feature_obj(feature_obj)
            elif entity_type == 'point':
                res.output = 'point'
                point_obj = self.osm_bridge.get_point(ref)
                res.point = OBLWMToROSAdapter.get_point_msg_from_point_obj(point_obj)
            elif entity_type == 'shape':
                res.output = 'shape'
                shape_obj = self.osm_bridge.get_shape(ref)
                res.shape = OBLWMToROSAdapter.get_shape_msg_from_shape_obj(shape_obj)

            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    
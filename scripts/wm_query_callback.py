from osm_bridge_ros_wrapper.msg import *

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
                res.building = self._get_building_msg_from_building_obj(building_obj)
            elif entity_type == 'floor':
                res.output = 'floor'
                floor_obj = self.osm_bridge.get_floor(ref)
                res.floor = self._get_floor_msg_from_floor_obj(floor_obj)
            elif entity_type == 'elevator':
                res.output = 'elevator'
                elevator_obj = self.osm_bridge.get_elevator(ref)
                res.elevator = self._get_elevator_msg_from_elevator_obj(elevator_obj)
            elif entity_type == 'stairs':
                res.output = 'stairs'
                stairs_obj = self.osm_bridge.get_stairs(ref)
                res.stairs = self._get_stairs_msg_from_stairs_obj(stairs_obj)
            elif entity_type == 'room':
                res.output = 'room'
                room_obj = self.osm_bridge.get_room(ref)
                res.room = self._get_room_msg_from_room_obj(room_obj)
            elif entity_type == 'area':
                res.output = 'area'
                area_obj = self.osm_bridge.get_area(ref)
                res.area = self._get_area_msg_from_room_obj(room_obj)
            elif entity_type == 'corridor':
                res.output = 'corridor'
                corridor_obj = self.osm_bridge.get_corridor(ref)
                res.corridor = self._get_corridor_msg_from_corridor_obj(corridor_obj)
            elif entity_type == 'connection':
                res.output = 'connection'
                connection_obj = self.osm_bridge.get_connection(ref)
                res.connection = self._get_connection_msg_from_connection_obj(connection_obj)
            elif entity_type == 'local_area':
                res.output = 'local_area'
                local_area_obj = self.osm_bridge.get_local_area(ref)
                local_area_obj.geometry
                res.local_area = self._get_local_area_msg_from_local_area_obj(local_area_obj)
            elif entity_type == 'door':
                res.output = 'door'
                door_obj = self.osm_bridge.get_door(ref)
                res.door = self._get_door_msg_from_door_obj(door_obj)
            elif entity_type == 'wall':
                res.output = 'wall'
                wall_obj = self.osm_bridge.get_wall(ref)
                res.wall = self._get_wall_msg_from_wall_obj(wall_obj)
            elif entity_type == 'side':
                res.output = 'side'
                side_obj = self.osm_bridge.get_side(ref)
                res.side = self._get_side_msg_from_side_obj(side_obj)
            elif entity_type == 'feature':
                res.output = 'feature'
                feature_obj = self.osm_bridge.get_feature(ref)
                res.feature = self._get_feature_msg_from_feature_obj(feature_obj)
            elif entity_type == 'point':
                res.output = 'point'
                point_obj = self.osm_bridge.get_point(ref)
                res.point = self._get_point_msg_from_point_obj(point_obj)
            elif entity_type == 'shape':
                res.output = 'shape'
                shape_obj = self.osm_bridge.get_shape(ref)
                res.shape = self._get_shape_msg_from_shape_obj(shape_obj)

            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def _get_building_msg_from_building_obj(self, building_obj) :
        building = Building()
        building.id = building_obj.id
        building.ref = building_obj.ref
        building.floor_ids = building_obj._floor_ids if building_obj._floor_ids is not None else []
        building.elevator_ids = building_obj._elevator_ids if building_obj._elevator_ids is not None else []
        building.stairs_ids = building_obj._stairs_ids if building_obj._stairs_ids is not None else []
        return building

    def _get_floor_msg_from_floor_obj(self, floor_obj) :
        floor = Floor()
        floor.id = floor_obj.id
        floor.ref = floor_obj.ref
        floor.room_ids = floor_obj._room_ids if floor_obj._room_ids is not None else []
        floor.area_ids = floor_obj._area_ids if floor_obj._area_ids is not None else []
        floor.corridor_ids = floor_obj._corridor_ids if floor_obj._corridor_ids is not None else []
        floor.connection_ids = floor_obj._connection_ids if floor_obj._connection_ids is not None else []
        floor.wall_ids = floor_obj._wall_ids if floor_obj._wall_ids is not None else []
        floor.feature_ids = floor_obj._feature_ids if floor_obj._feature_ids is not None else []
        return floor

    def _get_elevator_msg_from_elevator_obj(self, elevator_obj) :
        elevator = Elevator()
        elevator.id = elevator_obj.id
        elevator.levels = elevator_obj.levels if elevator_obj.levels is not None else []
        elevator.wall_ids = elevator_obj._wall_ids if elevator_obj._wall_ids is not None else []
        elevator.door_ids = elevator_obj._door_ids if elevator_obj._door_ids is not None else []
        elevator.connection_ids = elevator_obj._connection_ids if elevator_obj._connection_ids is not None else []
        elevator.topology_id = elevator_obj._topology_id if elevator_obj._topology_id is not None else []
        elevator.local_area_ids = elevator_obj._local_area_ids if elevator_obj._local_area_ids is not None else []
        elevator.shape_id = elevator_obj._geometry_id if elevator_obj._geometry_id is not None else []
        return elevator

    def _get_stairs_msg_from_stairs_obj(self, stairs_obj) :
        stairs = Stairs()
        stairs.id = stairs_obj.id
        stairs.levels = stairs_obj.levels if stairs_obj.levels is not None else []
        stairs.wall_ids = stairs_obj._wall_ids if stairs_obj._wall_ids is not None else []
        stairs.door_ids = stairs_obj._door_ids if stairs_obj._door_ids is not None else []
        stairs.connection_ids = stairs_obj._connection_ids if stairs_obj._connection_ids is not None else []
        stairs.topology_id = stairs_obj._topology_id if stairs_obj._topology_id is not None else []
        stairs.local_area_ids = stairs_obj._local_area_ids if stairs_obj._local_area_ids is not None else []
        stairs.shape_id = stairs_obj._geometry_id if stairs_obj._shape_ids is not None else []
        return stairs

    def _get_room_msg_from_room_obj(self, room_obj) :
        room = Room()
        room.id = room_obj.id
        room.level = room_obj.level if room_obj.level is not None else []
        room.wall_ids = room_obj._wall_ids if room_obj._wall_ids is not None else []
        room.door_ids = room_obj._door_ids if room_obj._door_ids is not None else []
        room.connection_ids = room_obj._connection_ids if room_obj._connection_ids is not None else []
        room.topology_id = room_obj._topology_id if room_obj._topology_id is not None else []
        room.local_area_ids = room_obj._local_area_ids if room_obj._local_area_ids is not None else []
        room.shape_id = room_obj._geometry_id if room_obj._geometry_id is not None else []
        return room

    def _get_corridor_msg_from_corridor_obj(self, corridor_obj) :
        corridor = Corridor()
        corridor.id = corridor_obj.id
        corridor.level = corridor_obj.level if corridor_obj.level is not None else []
        corridor.wall_ids = corridor_obj._wall_ids if corridor_obj._wall_ids is not None else []
        corridor.door_ids = corridor_obj._door_ids if corridor_obj._door_ids is not None else []
        corridor.connection_ids = corridor_obj._connection_ids if corridor_obj._connection_ids is not None else []
        corridor.topology_id = corridor_obj._topology_id if corridor_obj._topology_id is not None else []
        corridor.local_area_ids = corridor_obj._local_area_ids if corridor_obj._local_area_ids is not None else []
        corridor.shape_id = corridor_obj._geometry_id if corridor_obj._geometry_id is not None else []
        return corridor

    def _get_corridor_msg_from_area_obj(self, area_obj) :
        area = Area()
        area.id = area_obj.id
        area.level = area_obj.level if area_obj.level is not None else []
        area.wall_ids = area_obj._wall_ids if area_obj._wall_ids is not None else []
        area.door_ids = area_obj._door_ids if area_obj._door_ids is not None else []
        area.connection_ids = area_obj._connection_ids if area_obj._connection_ids is not None else []
        area.topology_id = area_obj._topology_id if area_obj._topology_id is not None else []
        area.local_area_ids = area_obj._local_area_ids if area_obj._local_area_ids is not None else []
        area_obj.shape_id = area_obj._shape_ids if area_obj._geometry_id is not None else []
        return area

    def _get_local_area_msg_from_local_area_obj(self, local_area_obj) :
        local_area = LocalArea()
        local_area.id = local_area_obj.id
        local_area.ref = local_area_obj.ref if local_area_obj.ref is not None else ''
        local_area.behaviour = local_area_obj.behaviour if local_area_obj.behaviour is not None else ''
        local_area.topology_id = local_area_obj._topology_id
        local_area.shape_id = local_area_obj._geometry_id
        return local_area

    def _get_door_msg_from_door_obj(self, door_obj) :
        door = Door()
        door.id = door_obj.id
        door.level = door_obj.level
        door.side_ids = door_obj._side_ids if door_obj._side_ids is not None else []
        door.shape_id = door_obj._geometry_id 
        door.topology_id = door_obj._topology_id
        return door

    def _get_wall_msg_from_wall_obj(self, wall_obj) :
        wall = Wall()
        wall.id = wall_obj.id
        wall.level = wall_obj.level
        wall.side_ids = wall_obj._side_ids if wall_obj._side_ids is not None else []
        wall.shape_id = wall_obj._geometry_id 
        return wall

    def _get_side_msg_from_side_obj(self, side_obj) :
        side = Side()
        side.texture = side_obj.texture if side_obj.texture is not None else ''
        side.paint = side_obj.paint if side_obj.paint is not None else ''
        side.corner_ids = side_obj._corner_ids if side_obj._corner_ids is not None else []
        side.feature_ids = side_obj._feature_ids if side_obj._feature_ids is not None else []
        return side

    def _get_feature_msg_from_feature_obj(self, feature_obj) :
        feature = Feature()
        feature.id = feature_obj.id
        feature.height = feature_obj.height if feature_obj.height is not None else -1
        feature.width = feature_obj.width if feature_obj.width is not None else -1
        feature.breast = feature_obj.breast if feature_obj.breast is not None else -1
        feature.level = feature_obj.level if feature_obj.level is not None else -1
        feature.point = self._get_point_msg_from_point_obj(feature.point)
        return feature

    def _get_point_msg_from_point_obj(self, point_obj) :
        point = Point()
        point.id = point_obj.id
        point.x = point_obj.x if point_obj.x is not None else -1
        point.y = point_obj.y if point_obj.y is not None else -1
        return point

    def _get_shape_msg_from_shape_obj(self, shape_obj) :
        shape = Shape()
        shape.points = []
        for pt_obj in shape_obj.points:
            shape.points.append(self._get_point_msg_from_point_obj(pt_obj))
        return shape

    
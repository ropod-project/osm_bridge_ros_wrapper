from osm_bridge_ros_wrapper.msg import *

class OBLWMToROSAdapter(object):

    @staticmethod
    def get_building_msg_from_building_obj(building_obj) :
        building = Building()
        building.id = building_obj.id
        building.ref = OBLWMToROSAdapter._convert_to_string(building_obj.ref)
        building.floor_ids = building_obj._floor_ids if building_obj._floor_ids is not None else []
        building.elevator_ids = building_obj._elevator_ids if building_obj._elevator_ids is not None else []
        building.stairs_ids = building_obj._stairs_ids if building_obj._stairs_ids is not None else []
        return building

    @staticmethod
    def get_floor_msg_from_floor_obj(floor_obj) :
        floor = Floor()
        floor.id = floor_obj.id
        floor.ref = OBLWMToROSAdapter._convert_to_string(floor_obj.ref)
        floor.room_ids = floor_obj._room_ids if floor_obj._room_ids is not None else []
        floor.area_ids = floor_obj._area_ids if floor_obj._area_ids is not None else []
        floor.corridor_ids = floor_obj._corridor_ids if floor_obj._corridor_ids is not None else []
        floor.connection_ids = floor_obj._connection_ids if floor_obj._connection_ids is not None else []
        floor.wall_ids = floor_obj._wall_ids if floor_obj._wall_ids is not None else []
        floor.feature_ids = floor_obj._feature_ids if floor_obj._feature_ids is not None else []
        return floor

    @staticmethod
    def get_elevator_msg_from_elevator_obj(elevator_obj) :
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

    @staticmethod
    def get_stairs_msg_from_stairs_obj(stairs_obj) :
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

    @staticmethod
    def get_room_msg_from_room_obj(room_obj) :
        room = Room()
        room.id = room_obj.id
        room.ref = OBLWMToROSAdapter._convert_to_string(room_obj.ref)
        room.level = OBLWMToROSAdapter._convert_to_int(room_obj.level)
        room.wall_ids = room_obj._wall_ids if room_obj._wall_ids is not None else []
        room.door_ids = room_obj._door_ids if room_obj._door_ids is not None else []
        room.connection_ids = room_obj._connection_ids if room_obj._connection_ids is not None else []
        room.topology_id = room_obj._topology_id if room_obj._topology_id is not None else []
        room.local_area_ids = room_obj._local_area_ids if room_obj._local_area_ids is not None else []
        room.shape_id = room_obj._geometry_id if room_obj._geometry_id is not None else []
        room.amenity = OBLWMToROSAdapter._convert_to_string(room_obj.amenity)
        return room

    @staticmethod
    def get_corridor_msg_from_corridor_obj(corridor_obj) :
        corridor = Corridor()
        corridor.id = corridor_obj.id
        corridor.ref = OBLWMToROSAdapter._convert_to_string(corridor_obj.ref)
        corridor.level = OBLWMToROSAdapter._convert_to_int(corridor_obj.level)
        corridor.wall_ids = corridor_obj._wall_ids if corridor_obj._wall_ids is not None else []
        corridor.door_ids = corridor_obj._door_ids if corridor_obj._door_ids is not None else []
        corridor.connection_ids = corridor_obj._connection_ids if corridor_obj._connection_ids is not None else []
        corridor.topology_id = corridor_obj._topology_id if corridor_obj._topology_id is not None else []
        corridor.local_area_ids = corridor_obj._local_area_ids if corridor_obj._local_area_ids is not None else []
        corridor.shape_id = corridor_obj._geometry_id if corridor_obj._geometry_id is not None else []
        return corridor

    @staticmethod
    def get_area_msg_from_area_obj(area_obj) :
        area = Area()
        area.id = area_obj.id
        area.ref = OBLWMToROSAdapter._convert_to_string(area_obj.ref)
        area.level = OBLWMToROSAdapter._convert_to_int(area_obj.level)
        area.wall_ids = area_obj._wall_ids if area_obj._wall_ids is not None else []
        area.door_ids = area_obj._door_ids if area_obj._door_ids is not None else []
        area.connection_ids = area_obj._connection_ids if area_obj._connection_ids is not None else []
        area.topology_id = area_obj._topology_id if area_obj._topology_id is not None else []
        area.local_area_ids = area_obj._local_area_ids if area_obj._local_area_ids is not None else []
        area_obj.shape_id = area_obj._shape_ids if area_obj._geometry_id is not None else []
        return area

    @staticmethod
    def get_local_area_msg_from_local_area_obj(local_area_obj) :
        local_area = LocalArea()
        local_area.id = local_area_obj.id
        local_area.ref = OBLWMToROSAdapter._convert_to_string(local_area_obj.ref)
        local_area.behaviour = local_area_obj.behaviour if local_area_obj.behaviour is not None else ''
        local_area.topology_id = local_area_obj._topology_id
        local_area.shape_id = local_area_obj._geometry_id
        return local_area

    @staticmethod
    def get_door_msg_from_door_obj(door_obj) :
        door = Door()
        door.id = door_obj.id
        door.level = OBLWMToROSAdapter._convert_to_int(door_obj.level)
        door.side_ids = door_obj._side_ids if door_obj._side_ids is not None else []
        door.shape_id = door_obj._geometry_id 
        door.topology_id = door_obj._topology_id
        return door

    @staticmethod
    def get_wall_msg_from_wall_obj(wall_obj) :
        wall = Wall()
        wall.id = wall_obj.id
        wall.level = OBLWMToROSAdapter._convert_to_int(wall_obj.level)
        wall.side_ids = wall_obj._side_ids if wall_obj._side_ids is not None else []
        wall.shape_id = wall_obj._geometry_id 
        return wall

    @staticmethod
    def get_side_msg_from_side_obj(side_obj) :
        side = Side()
        side.texture = OBLWMToROSAdapter._convert_to_string(side_obj.texture)
        side.paint = OBLWMToROSAdapter._convert_to_string(side_obj.paint)
        side.corner_ids = side_obj._corner_ids if side_obj._corner_ids is not None else []
        side.feature_ids = side_obj._feature_ids if side_obj._feature_ids is not None else []
        return side

    @staticmethod
    def get_feature_msg_from_feature_obj(feature_obj) :
        feature = Feature()
        feature.id = feature_obj.id
        feature.height = OBLWMToROSAdapter._convert_to_decimal(feature_obj.height)
        feature.width = OBLWMToROSAdapter._convert_to_decimal(feature_obj.width)
        feature.breast = OBLWMToROSAdapter._convert_to_decimal(feature_obj.breast)
        feature.level = OBLWMToROSAdapter._convert_to_int(feature_obj.level)
        feature.point = OBLWMToROSAdapter.get_point_msg_from_point_obj(feature.point)
        return feature

    @staticmethod
    def get_point_msg_from_point_obj(point_obj) :
        point = Point()
        point.id = point_obj.id
        point.x = point_obj.x
        point.y = point_obj.y
        return point

    @staticmethod
    def get_shape_msg_from_shape_obj(shape_obj) :
        shape = Shape()
        shape.points = []
        for pt_obj in shape_obj.points:
            shape.points.append(OBLWMToROSAdapter.get_point_msg_from_point_obj(pt_obj))
        return shape

    @staticmethod
    def get_planner_area(planner_area_object):
        planner_area = PlannerArea()
        if planner_area_object.type == 'room':
            planner_area.room = OBLWMToROSAdapter.get_room_msg_from_room_obj(planner_area_object)
        elif planner_area_object.type == 'corridor' or planner_area_object.type == 'junction':
            planner_area.corridor = OBLWMToROSAdapter.get_corridor_msg_from_corridor_obj(planner_area_object)
        elif planner_area_object.type == 'area':
            planner_area.area = OBLWMToROSAdapter.get_area_msg_from_area_obj(planner_area_object)
        elif planner_area_object.type == 'elevator':
            planner_area.elevator = OBLWMToROSAdapter.get_elevator_msg_from_elevator_obj(planner_area_object)
        elif planner_area_object.type == 'stairs':
            planner_area.stairs = OBLWMToROSAdapter.get_stairs_msg_from_stairs_obj(planner_area_object)

        planner_area.area_type = OBLWMToROSAdapter._convert_to_string(planner_area_object.type)

        for nav_area in planner_area_object.navigation_areas:
            planner_area.navigation_areas.append(OBLWMToROSAdapter.get_local_area_msg_from_local_area_obj(nav_area))

        if planner_area_object.exit_door:
            planner_area.exit_door = OBLWMToROSAdapter.get_door_msg_from_door_obj(planner_area_object.exit_door)

        return planner_area

    @staticmethod
    def _convert_to_int(string):
        if string is not None:
            return int(string) if string.strip() else -1
        else:
            return -1

    @staticmethod
    def _convert_to_string(string):
        if string is not None:
            return str(string) if string.strip() else ''
        else:
            return ''
    @staticmethod
    def _convert_to_decimal(string):
        if string is not None:
            return float(string) if string.strip() else -1.0
        else:
            return -1.0

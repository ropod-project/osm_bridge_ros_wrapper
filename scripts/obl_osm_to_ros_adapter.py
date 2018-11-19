from osm_bridge_ros_wrapper.msg import *

class OBLOSMToROSAdapter(object):

    @staticmethod
    def get_node_msg_from_node_obj(node_obj) :
        node = Node()
        node.id = node_obj.id
        node.lat = node_obj.lat
        node.lon = node_obj.lon
        node.tags = OBLOSMToROSAdapter.get_tag_msg_list_from_tag_obj_list(node_obj.tags)
        return node

    @staticmethod
    def get_way_msg_from_way_obj(way_obj) :
        way = Way()
        way.id = way_obj.id
        way.node_ids = way_obj.nodes
        way.tags = OBLOSMToROSAdapter.get_tag_msg_list_from_tag_obj_list(way_obj.tags)
        return way

    @staticmethod
    def get_relation_msg_from_relation_obj(rel_obj) :
        relation = Relation()
        relation.id = rel_obj.id
        relation.tags = OBLOSMToROSAdapter.get_tag_msg_list_from_tag_obj_list(rel_obj.tags)
        relation.members = OBLOSMToROSAdapter.get_member_msg_list_from_member_obj_list(rel_obj.members)
        return relation

    @staticmethod
    def get_tag_msg_list_from_tag_obj_list(tags_obj_list) :
        tags = []
        for i in tags_obj_list :
            tags.append(Tag(key=str(i.key), value=str(i.value)))
        return tags

    @staticmethod
    def get_member_msg_list_from_member_obj_list(member_obj_list) :
        members = []
        for i in member_obj_list :
            members.append(Member(ref=i.ref, role=str(i.role), type=str(i.type)))
        return members
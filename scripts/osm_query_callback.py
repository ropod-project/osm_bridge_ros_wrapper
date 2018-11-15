from osm_bridge_ros_wrapper.msg import *

class OSMQueryCallback(object):

    """callback for osm query server"""

    def __init__(self, osm_adapter):
        """TODO: to be defined1. """
        self.osm_adapter = osm_adapter

    def get_response(self, req):
        ids = req.ids
        data_type = req.type
        role = req.role
        role_type = req.role_type
        scope_id = req.scope_id
        scope_role = req.scope_role
        scope_role_type = req.scope_role_type
        tags_list = req.tags
        query = req.query
        tags = {}
        for i in tags_list :
            tags[i.key] = i.value
        nodes = []
        ways = []
        relations = []
        if len(ids) > 0 :
            nodes, ways, relations = self.osm_adapter.get_osm_element_by_id(
                    ids=ids, data_type=data_type, role=role, role_type=role_type)
        elif len(tags.keys()) > 0 :
            nodes, ways, relations = self.osm_adapter.search_by_tag(
                    data_type=data_type, scope_id=scope_id, scope_role=scope_role,
                    scope_role_type=scope_role_type, key_val_dict=tags)
        elif len(query) > 0 :
            nodes, ways, relations = self.osm_adapter.get(query)
        res = OSMQueryResult()
        for i in nodes:
            res.nodes.append(self._get_node_msg_from_node_obj(i))
        for i in ways:
            res.ways.append(self._get_way_msg_from_way_obj(i))
        for i in relations :
            res.relations.append(self._get_relation_msg_from_relation_obj(i))
        return res


    def _get_node_msg_from_node_obj(self, node_obj) :
        node = Node()
        node.id = node_obj.id
        node.lat = node_obj.lat
        node.lon = node_obj.lon
        node.tags = self._get_tag_msg_list_from_tag_obj_list(node_obj.tags)
        return node

    def _get_way_msg_from_way_obj(self, way_obj) :
        way = Way()
        way.id = way_obj.id
        way.node_ids = way_obj.nodes
        way.tags = self._get_tag_msg_list_from_tag_obj_list(way_obj.tags)
        return way

    def _get_relation_msg_from_relation_obj(self, rel_obj) :
        relation = Relation()
        relation.id = rel_obj.id
        relation.tags = self._get_tag_msg_list_from_tag_obj_list(rel_obj.tags)
        relation.members = self._get_member_msg_list_from_member_obj_list(rel_obj.members)
        return relation

    def _get_tag_msg_list_from_tag_obj_list(self, tags_obj_list) :
        tags = []
        for i in tags_obj_list :
            tags.append(Tag(key=i.key, value=i.value))
        return tags

    def _get_member_msg_list_from_member_obj_list(self, member_obj_list) :
        members = []
        for i in member_obj_list :
            members.append(Member(ref=i.ref, role=i.role, type=i.type))
        return members

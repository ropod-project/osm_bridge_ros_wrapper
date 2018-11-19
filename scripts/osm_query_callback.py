from osm_bridge_ros_wrapper.msg import *
from obl_osm_to_ros_adapter import OBLOSMToROSAdapter 

class OSMQueryCallback(object):

    """callback for osm query server"""

    def __init__(self, osm_adapter):
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
        for node in nodes:
            res.nodes.append(OBLOSMToROSAdapter.get_node_msg_from_node_obj(node))
        for way in ways:
            res.ways.append(OBLOSMToROSAdapter.get_way_msg_from_way_obj(way))
        for relation in relations :
            res.relations.append(OBLOSMToROSAdapter.get_relation_msg_from_relation_obj(relation))
        return res

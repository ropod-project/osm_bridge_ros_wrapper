from osm_bridge_ros_wrapper.msg import *
from obl_wm_to_ros_adapter import OBLWMToROSAdapter
import rospy

class SemanticFeaturesCallback(object):

    """callback for semantic features server"""

    def __init__(self, semantic_feature_finder):
        self.semantic_feature_finder = semantic_feature_finder

    def get_safe_response(self, req):
        ref = req.id or req.ref
        try:
            res = self._get_response(ref)
            return res
        except Exception as e:
            rospy.logerr(str(e))
            return None

    def _get_response(self, ref):
        res = SemanticFeaturesResult()
        semantic_features_obj = self.semantic_feature_finder.get_features(ref)
        semantic_features_ros = OBLWMToROSAdapter.get_semantic_features_ros_from_semantic_features_obj(semantic_features_obj)

        res.wall_sides = semantic_features_ros[0]
        res.door_sides = semantic_features_ros[1]
        res.features = semantic_features_ros[2]
        res.pillars = semantic_features_ros[3]
        return res
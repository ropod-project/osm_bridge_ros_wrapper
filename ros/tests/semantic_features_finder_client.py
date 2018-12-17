#!/usr/bin/env python

PACKAGE = 'osm_bridge_ros_wrapper'
NODE = 'semantic_features_client'

import rospy
from osm_bridge_ros_wrapper.msg import *
from actionlib import SimpleActionClient

class SemanticFeaturesClient(object):

    """A test for osm_bridge_ros.py 's semantic_features_server"""

    def __init__(self):
        SERVER = "/semantic_features"
        self.client = SimpleActionClient(SERVER, SemanticFeaturesAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("Connected to server")

        req = SemanticFeaturesGoal(ref='BRSU_C_L0_RoomC022')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        
        rospy.signal_shutdown("Semantic features server tests complete")

    def done_cb(self, status, result):

        try:
            assert(len(result.wall_sides) == 5)
            assert(len(result.door_sides) == 2)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")


if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = SemanticFeaturesClient()
    rospy.spin()

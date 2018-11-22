#!/usr/bin/env python

PACKAGE = 'osm_bridge_ros_wrapper'
NODE = 'path_planner_client'

import rospy
from osm_bridge_ros_wrapper.msg import *
from actionlib import SimpleActionClient

class PathPlannerClient(object):

    """A test for osm_bridge_ros.py 's wm_query_server"""

    def __init__(self):
        SERVER = "/path_planner"
        self.client = SimpleActionClient(SERVER, PathPlannerAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("Connected to server")

        req = PathPlannerGoal(start_floor='AMK_L-1', destination_floor='AMK_L4',start_area='AMK_D_L-1_C41',destination_area='AMK_B_L4_C1', start_local_area='AMK_D_L-1_C41_LA1',destination_local_area='AMK_B_L4_C1_LA1')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        rospy.signal_shutdown("Path planner server tests complete")

    def done_cb(self, status, result):
        # rospy.loginfo(status)
        # rospy.loginfo(result)
        try:
            assert(len(result.planner_areas) > 0)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")



if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = PathPlannerClient()
    rospy.spin()

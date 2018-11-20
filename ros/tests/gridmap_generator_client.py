#!/usr/bin/env python

PACKAGE = 'osm_bridge_ros_wrapper'
NODE = 'gridmap_generator_client'

import rospy
import os
from osm_bridge_ros_wrapper.msg import *
from actionlib import SimpleActionClient

class GridmapGeneratorClient(object):

    """A test for osm_bridge_ros.py 's gridmap_generator_server"""

    def __init__(self):
        rospy.loginfo("inside __init__ of OSMClient")
        SERVER = "/grid_map_generator"
        self.client = SimpleActionClient(SERVER, GridMapGeneratorAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("connected to server")

        req = GridMapGeneratorGoal(floor=4, local_offset_x=25, local_offset_y=50)
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        rospy.signal_shutdown("Test complete")

    def done_cb(self, status, result):
        rospy.loginfo(status)
        rospy.loginfo(result)
        try:
            assert(os.path.isfile(result.filename))
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")



if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = GridmapGeneratorClient()
    rospy.spin()

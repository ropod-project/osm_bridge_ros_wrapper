#!/usr/bin/env python

PACKAGE = 'osm_bridge_ros_wrapper'
NODE = 'nearest_wlan_finder_client'

import rospy
import os
from osm_bridge_ros_wrapper.msg import *
from actionlib import SimpleActionClient

class NearestWLANFinderClient(object):

    """A test for osm_bridge_ros.py 's nearest_wlan_finder action"""

    def __init__(self):
        rospy.loginfo("inside __init__ of NearestWLANFinderClient")
        SERVER = "/nearest_wlan"
        self.client = SimpleActionClient(SERVER, NearestWLANAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to server")

        req = NearestWLANGoal(x=0.0, y=5.0, is_x_y_provided=True, floor='BRSU_L0')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(x=0.0, y=5.0, is_x_y_provided=True, floor='127')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(is_x_y_provided=False, area='BRSU_A_L0_A1')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(is_x_y_provided=False, area='125')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(is_x_y_provided=False, local_area='BRSU_A_L0_A1_LA1')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(is_x_y_provided=False, local_area='120')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        rospy.signal_shutdown("Test complete")

    def done_cb(self, status, result):
        # rospy.loginfo(status)
        # rospy.loginfo(result)
        try:
            assert(result.point.id == 2658)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")



if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = NearestWLANFinderClient()
    rospy.spin()

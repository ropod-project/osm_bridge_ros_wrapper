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

        req = NearestWLANGoal(x=0.0, y=5.0, type=NearestWLANGoal.X_Y_AND_FLOOR, ref='BRSU_L0')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(x=0.0, y=5.0, type=NearestWLANGoal.X_Y_AND_FLOOR, id=127)
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(type=NearestWLANGoal.AREA, ref='BRSU_A_L0_A1')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(type=NearestWLANGoal.AREA, id=125)
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(type=NearestWLANGoal.LOCAL_AREA, ref='BRSU_A_L0_A1_LA1')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = NearestWLANGoal(type=NearestWLANGoal.LOCAL_AREA, id=120)
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

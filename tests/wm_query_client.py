#!/usr/bin/env python

PACKAGE = 'osm_bridge_ros_wrapper'
NODE = 'wm_query_client'

import rospy
from osm_bridge_ros_wrapper.msg import *
from actionlib import SimpleActionClient

class WMQueryClient(object):

    """A test for osm_bridge_ros.py 's wm_query_server"""

    def __init__(self):
        SERVER = "/wm_query"
        self.client = SimpleActionClient(SERVER, WMQueryAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("Connected to server")

        req = WMQueryGoal(id=149, type="building")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(id=164, type="floor")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(id=5, type="elevator")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(id=140, type="corridor")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(ref='AMK_B_L-1_C14', type="corridor")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(id=22, type="room")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(id=173, type="local_area")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(id=161, type="door")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(id=4865, type="point")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        req = WMQueryGoal(id=1021, type="shape")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()

        rospy.signal_shutdown("WM Query server tests complete")

    def done_cb(self, status, result):
        # rospy.loginfo(status)
        # rospy.loginfo(result)
        try:
            assert(result.output != None)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")



if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = WMQueryClient()
    rospy.spin()

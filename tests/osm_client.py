#!/usr/bin/env python

PACKAGE = 'osm_bridge_ros_wrapper'
NODE = 'osm_client'

import rospy
from osm_bridge_ros_wrapper.msg import *
from actionlib import SimpleActionClient

class OSMClient(object):

    """A test for osm_bridge_ros.py 's osm_query_server"""

    def __init__(self):
        rospy.loginfo("inside __init__ of OSMClient")
        SERVER = "/osm_query"
        self.client = SimpleActionClient(SERVER, OSMQueryAction)
        connected = self.client.wait_for_server()
        rospy.loginfo("connected to server")

        req = OSMQueryGoal(ids=[4865], type="node")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(ids=[4865, 4864, 4866], type="node")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(ids=[499, 501], type="way")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(ids=[149], type="relation")
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(ids=[149],type='relation',role='geometry',role_type='way')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(ids=[], type='node',tags=[Tag(key='highway',value='elevator')])
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(ids=[], type='way',tags=[Tag(key='highway',value='footway')])
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(ids=[], type='way', tags=[Tag(key='level',value='-1'),Tag(key='indoor',value='wall')])
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(ids=[], type='relation',tags=[Tag(key='type',value='building')])
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        req = OSMQueryGoal(query='way[!"level"][indoor=wall];')
        self.client.send_goal(req, done_cb=self.done_cb)
        self.client.wait_for_result()
        rospy.signal_shutdown("Test complete")

    def done_cb(self, status, result):
#         rospy.loginfo(status)
#         rospy.loginfo(result)
        try:
            assert(len(result.nodes) > 0 or len(result.ways) > 0 or len(result.relations) > 0)
            rospy.loginfo("Test passed")
        except Exception as e:
            rospy.logerr("Test failed")



if __name__ == "__main__":
    rospy.init_node(NODE)
    tester = OSMClient()
    rospy.spin()

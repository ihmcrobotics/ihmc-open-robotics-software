#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: test_empty_service.py 3803 2009-02-11 02:04:39Z rob_wheeler $

PKG = 'rosjava'
NAME = 'string_passthrough'
import roslib; roslib.load_manifest(PKG)

from ros import rospy
from ros import std_msgs
from ros import rostest

import sys
import time
import unittest

from std_msgs.msg import String

class TestStringPassthrough(unittest.TestCase):
        
    def setUp(self):
        rospy.init_node(NAME)
        self.nodes = ['/node%s'%(i) for i in xrange(10)]
        self.nodes_set = set(self.nodes)
        
        # keep track of nodes that have done callbacks
        self.fixture_nodes_cb = set()
        self.test_nodes_cb = set()
        
        rospy.Subscriber('string_in', String, self.cb_from_fixture)
        rospy.Subscriber('string_out', String, self.cb_from_test)
        
    def cb_from_fixture(self, msg):
        self.fixture_nodes_cb.add(msg.data)

    def cb_from_test(self, msg):
        self.test_nodes_cb.add(msg.data)

    def test_string_passthrough(self):
        # 20 seconds to validate fixture
        timeout_t = time.time() + 20.
        print "waiting for 20 seconds for fixture to verify"
        while not self.fixture_nodes_cb == self.nodes_set and \
                not rospy.is_shutdown() and \
                timeout_t > time.time():
            time.sleep(0.2)

        self.failIf(timeout_t < time.time(), "timeout exceeded")
        self.failIf(rospy.is_shutdown(), "node shutdown")            
        self.assertEquals(self.nodes_set, self.fixture_nodes_cb, "fixture did not validate: %s vs %s"%(self.nodes_set, self.fixture_nodes_cb))

        # another 20 seconds to validate client
        timeout_t = time.time() + 20.
        print "waiting for 20 seconds for client to verify"
        while not self.test_nodes_cb == self.nodes_set and \
                not rospy.is_shutdown() and \
                timeout_t > time.time():
            time.sleep(0.2)

        self.assertEquals(self.nodes_set, self.test_nodes_cb, "passthrough did not pass along all message")

        # Create a new Publisher here.  This will validate publisherUpdate()
        pub = rospy.Publisher('string_in', String)
        msg = 'test_publisherUpdate'
        timeout_t = time.time() + 20.
        print "waiting for 20 seconds for client to verify"
        while not msg in self.nodes_set and \
                not rospy.is_shutdown() and \
                timeout_t > time.time():
            pub.publish(data=msg)
            time.sleep(0.2)

        

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestStringPassthrough, sys.argv)

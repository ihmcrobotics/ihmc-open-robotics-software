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
NAME = 'int64_passthrough'
import roslib; roslib.load_manifest(PKG)

from ros import rospy
from ros import std_msgs
from ros import rostest

import sys
import time
import unittest

from std_msgs.msg import Int64

class TestInt64Passthrough(unittest.TestCase):
        
    def setUp(self):
        rospy.init_node(NAME)
        
        # keep track of Nth and (N-1)th message
        self.fixture_prev = self.fixture_curr = None
        self.test_prev = self.test_curr = None
        
        rospy.Subscriber('int64_in', Int64, self.cb_from_fixture)
        rospy.Subscriber('int64_out', Int64, self.cb_from_test)
        
    def cb_from_fixture(self, msg):
        self.fixture_prev = self.fixture_curr
        self.fixture_curr = msg

    def cb_from_test(self, msg):
        self.test_prev = self.test_curr
        self.test_curr = msg

    def test_int64_passthrough(self):
        # 20 seconds to validate fixture
        timeout_t = time.time() + 20.
        print "waiting for 20 seconds for fixture to verify"
        while self.fixture_prev is None and \
                not rospy.is_shutdown() and \
                timeout_t > time.time():
            time.sleep(0.2)

        self.failIf(timeout_t < time.time(), "timeout exceeded")
        self.failIf(rospy.is_shutdown(), "node shutdown")            
        self.failIf(self.fixture_prev is None, "no data from fixture")
        self.failIf(self.fixture_curr is None, "no data from fixture")
        self.assertEquals(self.fixture_prev.data + 1, self.fixture_curr.data, "fixture should incr by one")

        # another 20 seconds to validate client
        timeout_t = time.time() + 20.
        print "waiting for 20 seconds for client to verify"
        while self.test_prev is None and \
                not rospy.is_shutdown() and \
                timeout_t > time.time():
            time.sleep(0.2)

        self.failIf(self.test_prev is None, "no data from test")
        self.assertEquals(self.test_prev.data + 1, self.test_curr.data, "test does not appear to match fixture data")


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestInt64Passthrough, sys.argv)

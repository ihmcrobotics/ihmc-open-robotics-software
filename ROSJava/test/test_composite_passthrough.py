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
NAME = 'composite_passthrough'
import roslib; roslib.load_manifest(PKG)

from ros import rospy
from ros import test_ros
from ros import rostest

import sys
import time
import unittest

from test_ros.msg import Composite

class CompositePassthrough(unittest.TestCase):
        
    def setUp(self):
        rospy.init_node(NAME)
        
        self.fixture_curr = None
        self.test_curr = None
        
        rospy.Subscriber('composite_in', Composite, self.cb_from_fixture)
        rospy.Subscriber('composite_out', Composite, self.cb_from_test)
        
    def cb_from_fixture(self, msg):
        self.fixture_curr = msg

    def cb_from_test(self, msg):
        self.test_curr = msg

    def test_composite_passthrough(self):
        # 20 seconds to validate fixture
        timeout_t = time.time() + 20.
        print "waiting for 20 seconds for fixture to verify"
        while self.fixture_curr is None and \
                not rospy.is_shutdown() and \
                timeout_t > time.time():
            time.sleep(0.2)

        self.failIf(timeout_t < time.time(), "timeout exceeded")
        self.failIf(rospy.is_shutdown(), "node shutdown")            
        self.failIf(self.fixture_curr is None, "no data from fixture")
        m = self.fixture_curr
        self.assertAlmostEquals(m.a.x, m.b.x)
        self.assertAlmostEquals(m.a.y, m.b.y)
        self.assertAlmostEquals(m.a.z, m.b.z)

        # another 20 seconds to validate client
        timeout_t = time.time() + 20.
        print "waiting for 20 seconds for client to verify"
        while self.test_curr is None and \
                not rospy.is_shutdown() and \
                timeout_t > time.time():
            time.sleep(0.2)

        self.failIf(self.test_curr is None, "no data from test")
        m = self.test_curr
        self.assertAlmostEquals(m.a.x, m.b.x)
        self.assertAlmostEquals(m.a.y, m.b.y)
        self.assertAlmostEquals(m.a.z, m.b.z)

        # a.w = a.x + a.y + a.z.  Just make sure we're in the ballpark
        a = self.test_curr.a
        self.assert_(abs(a.x + a.y + a.z - a.w) < 10.)

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, CompositePassthrough, sys.argv)

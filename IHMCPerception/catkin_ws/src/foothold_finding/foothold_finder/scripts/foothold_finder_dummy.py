#!/usr/bin/env python

import roslib
from bzrlib.transport.http._urllib2_wrappers import Response
roslib.load_manifest('foothold_adaptation')

import rospy
from foothold_adaption_msg.srv import *

def adapt_foothold(request):
    print "Adapting footholds"
    adaptedFoodholds = request.initialFoodholds
    for foothold in adaptedFoodholds.footholds:
        if foothold.type.data == "LEFT":
            foothold.pose.position.z = 0.4
        
    response = AdaptFootholdsResponse(adaptedFoodholds)
    print response
    return response

def foothold_adaption_node():
    rospy.init_node('foothold_adaptation')
    service = rospy.Service('foothold_adaption/adapt', AdaptFootholds, adapt_foothold)
    print "Ready to adapt footholds."
    rospy.spin()

if __name__ == "__main__":
    foothold_adaption_node()

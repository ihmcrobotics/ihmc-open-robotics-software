#!/usr/bin/env python

import roslib
roslib.load_manifest('foothold_finder')

import rospy
from foothold_finding_msg.srv import *

def adapt_foothold(request):
    print "Adapting footholds..."
    adaptedFootholds = request.initialFootholds
    
    # Add changes here for debugging purposes.
    for foothold in adaptedFootholds:
        foothold.flag = 2     
#         if foothold.type.data == "LEFT":
#             foothold.pose.position.z = 0.4

        
    response = AdaptFootholdsResponse(adaptedFootholds)
    print response
    return response

def foothold_adaption_node():
    rospy.init_node('foothold_finder_dummy')
    service = rospy.Service('foothold_finder/adapt', AdaptFootholds, adapt_foothold)
    print "Ready to adapt footholds."
    rospy.spin()

if __name__ == "__main__":
    foothold_adaption_node()

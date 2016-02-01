#!/usr/bin/env python

import roslib
roslib.load_manifest('foothold_finder')

import rospy
from foothold_finding_msg.srv import *
import foothold_finding_msg.msg
import geometry_msgs.msg
import std_msgs.msg

def generate_footholds():
    print "Generating footholds"
    
    # Atlas
#     header = std_msgs.msg.Header(0, rospy.get_rostime(), "world")
#     stepnumber = 1
#     type =  std_msgs.msg.String("LEFT")
#     position = geometry_msgs.msg.Point(0.57, -0.1, 0.0)
#     orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)
#     pose = geometry_msgs.msg.Pose(position, orientation)
#     flag = 0
#     foothold = foothold_finding_msg.msg.Foothold(header, stepnumber, type, pose, flag)
#     footholds = [foothold]
    
    # StarlETH
    header = std_msgs.msg.Header(0, rospy.get_rostime(), "starleth/odometry")
    stepnumber = 1
    type =  std_msgs.msg.String("LF")
    position = geometry_msgs.msg.Point(0.910366654396, 0.120674788952, 0.0569435358047)
    #position = geometry_msgs.msg.Point(0.772246479988, 0.1186, 0.0)
    orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)
    pose = geometry_msgs.msg.Pose(position, orientation)
    flag = 0
    foothold = foothold_finding_msg.msg.Foothold(header, stepnumber, type, pose, flag)
    footholds = [foothold]
    
    return footholds

def call_foothold_finder(initialFootholds):
    rospy.wait_for_service('foothold_finder/adapt')
    try:
        adapt_footholds = rospy.ServiceProxy('foothold_finder/adapt', AdaptFootholds)
        response = adapt_footholds(AdaptFootholdsRequest(initialFootholds))
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def foothold_generator_test_node():
    rospy.init_node('foothold_generator_test')
    service = rospy.Service('foothold_adaption/adapt', AdaptFootholds, call_foothold_finder)
    initialFootholds = generate_footholds()
    print call_foothold_finder(initialFootholds)

if __name__ == "__main__":
    foothold_generator_test_node()

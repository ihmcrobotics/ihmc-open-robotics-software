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
    
    header = std_msgs.msg.Header(0, rospy.get_rostime(), "world")
    stepnumber = 0
    type =  std_msgs.msg.String("LEFT")
    position = geometry_msgs.msg.Point(-2.8, 2.8, 0.0)
    orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.924, 0.383)
    pose = geometry_msgs.msg.Pose(position, orientation)
    flag = 0
    foothold = foothold_finding_msg.msg.Foothold(header, stepnumber, type, pose, flag)
    footholds = [foothold]
    
#     header = std_msgs.msg.Header(0, rospy.get_rostime(), "world")
#     stepnumber = 0
#     type =  std_msgs.msg.String("LEFT")
#     position = geometry_msgs.msg.Point(-1.0, -1.0, 0.0)
#     orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)
#     pose = geometry_msgs.msg.Pose(position, orientation)
#     flag = 0
#     foothold = foothold_finding_msg.msg.Foothold(header, stepnumber, type, pose, flag)
#     footholds = [foothold]
#     
#     stepnumber = 1
#     type =  std_msgs.msg.String("RIGHT")
#     position = geometry_msgs.msg.Point(1.7, -1.1, 0.0)
#     orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)
#     pose = geometry_msgs.msg.Pose(position, orientation)
#     flag = 0
#     foothold = foothold_finding_msg.msg.Foothold(header, stepnumber, type, pose, flag)
#     footholds = footholds + [foothold]
#     
#     stepnumber = 2
#     type =  std_msgs.msg.String("LEFT")
#     position = geometry_msgs.msg.Point(0.5, -1.5, 0.0)
#     orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.383, 0.924)
#     pose = geometry_msgs.msg.Pose(position, orientation)
#     flag = 0
#     foothold = foothold_finding_msg.msg.Foothold(header, stepnumber, type, pose, flag)
#     footholds = footholds + [foothold]
#     
#     stepnumber = 3
#     type =  std_msgs.msg.String("RIGHT")
#     position = geometry_msgs.msg.Point(-1.5, -1.0, 0.0)
#     orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.707, 0.707)
#     pose = geometry_msgs.msg.Pose(position, orientation)
#     flag = 0
#     foothold = foothold_finding_msg.msg.Foothold(header, stepnumber, type, pose, flag)
#     footholds = footholds + [foothold]
    
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

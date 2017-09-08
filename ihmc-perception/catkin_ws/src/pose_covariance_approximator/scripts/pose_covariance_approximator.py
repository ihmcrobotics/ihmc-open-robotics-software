#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import std_msgs
from cmath import sqrt

# This is a nasty hack to get some values for an pose covariance of the robot
# and is by now ways correct. It was tuned based on experience from trials.
# Covariance is increased based on the travelled distance of the pelvis.
# Values for roll and pitch are fixed. Each state is assumed to be independent
# (no cross-correlation). Use this with extreme care!

def callback(newPose):
    global initialized, lastPoseWithCovariance

    if initialized == False:
        initialize(newPose)
        
    xFactor = 0.05 ** 2
    yFactor = 0.05 ** 2
    zFactor = 0.05 ** 2
    yawFactor = 0.25 ** 2
    
    positionDifference = geometry_msgs.msg.Vector3(0, 0, 0)
    positionDifference.x = newPose.pose.position.x - lastPoseWithCovariance.pose.position.x
    positionDifference.y = newPose.pose.position.y - lastPoseWithCovariance.pose.position.y
    positionDifference.z = newPose.pose.position.z - lastPoseWithCovariance.pose.position.z
    positionDifferenceNormSquare = positionDifference.x ** 2 + positionDifference.y ** 2 + positionDifference.z ** 2
    
    covariance = lastPoseWithCovariance.covariance
    covariance[0] =  covariance[0]  + xFactor * positionDifferenceNormSquare
    covariance[7] =  covariance[7]  + yFactor * positionDifferenceNormSquare
    covariance[14] = covariance[14] + zFactor * positionDifferenceNormSquare
    covariance[35] = covariance[35] + yawFactor * positionDifferenceNormSquare
    
    lastPoseWithCovariance = geometry_msgs.msg.PoseWithCovariance(newPose.pose, covariance)
    publisher.publish(geometry_msgs.msg.PoseWithCovarianceStamped(newPose.header, lastPoseWithCovariance))
    
def initialize(pose):
    global initialized, lastPoseWithCovariance
    covariance = [0, 0, 0, 0, 0, 0, 
                  0, 0, 0, 0, 0, 0, 
                  0, 0, 0, 0, 0, 0, 
                  0, 0, 0, 0.003, 0, 0, 
                  0, 0, 0, 0, 0.003, 0, 
                  0, 0, 0, 0, 0, 0.0]
    lastPoseWithCovariance = geometry_msgs.msg.PoseWithCovariance(pose.pose, covariance)
    initialized = True
    
#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global publisher, lastPoseWithCovariance, initialized
    initialized = False
    rospy.init_node('pose_covariance_approximator')
    rospy.Subscriber("/atlas/rootPose", geometry_msgs.msg.PoseStamped, callback)
    publisher = rospy.Publisher('/atlas/rootPoseWithCovariance', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass
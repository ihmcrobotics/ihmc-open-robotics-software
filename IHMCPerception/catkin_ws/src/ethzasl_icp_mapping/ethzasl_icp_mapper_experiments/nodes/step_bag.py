#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('modular_cloud_matcher')
import rospy
import rosbag
import tf
from sensor_msgs.msg import *


if len(sys.argv) < 3:
	sys.exit('Usage: %s bagName topicName <number of scan to jump> <list of scan to jump>' % sys.argv[0])

if len(sys.argv) == 3:
	jumpScan = 0
else:
	jumpScan = int(sys.argv[3])

blackList = []
if len(sys.argv) >= 5:
	blackList = sys.argv[4:]


bagName = sys.argv[1]
topicName = sys.argv[2]

rospy.init_node('republishBag')
pub = rospy.Publisher(topicName, PointCloud2)

bag = rosbag.Bag(bagName)

i = 0
for topic, msg, t in bag.read_messages(topics=[topicName, "/tf"]):
	if(topic == "/tf"):
		if(msg.transforms[0].header.frame_id == "/map"):
			if(msg.transforms[0].child_frame_id == "/odom"):
				br = tf.TransformBroadcaster()
				br.sendTransform((msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z), 
					(msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w),
					msg.transforms[0].header.stamp,
					"/odom",
					"/map")
	else:

		if str(i) in blackList:
			print "Scan " + str(i) + " is black-listed. Skipping it\n"

		elif (i >= jumpScan):
			print "----------------\nPoint cloud " + str(i)
			pointCloud = msg
			pub.publish(pointCloud)
			raw_input('Press Enter...\n')

		else:
			print "Skipping scan " + str(i)
		
		i = i + 1


bag.close()


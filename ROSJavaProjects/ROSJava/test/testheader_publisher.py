#!/usr/bin/env python
from ros import rospy
from ros import test_ros
import test_ros.msg

def publisher():
  rospy.init_node('testheader_publisher')
  pub = rospy.Publisher('test_header_in', test_ros.msg.TestHeader)
  r = rospy.Rate(10)
  m = test_ros.msg.TestHeader()
  m.caller_id = rospy.get_name()
  m.header.stamp = rospy.get_rostime()
  while not rospy.is_shutdown():
    pub.publish(m)
    r.sleep()

if __name__ == '__main__':
  publisher()

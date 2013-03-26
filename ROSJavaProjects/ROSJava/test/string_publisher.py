#!/usr/bin/env python
from ros import rospy
from ros import std_msgs
import std_msgs.msg

def publisher():
  rospy.init_node('string_publisher')
  pub = rospy.Publisher('string_in', std_msgs.msg.String)
  m = std_msgs.msg.String(rospy.get_name())
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    pub.publish(m)
    r.sleep()

if __name__ == '__main__':
  publisher()

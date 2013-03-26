#!/usr/bin/env python
from ros import rospy
from ros import std_msgs
import std_msgs.msg

def publisher():
  rospy.init_node('int64_publisher')
  pub = rospy.Publisher('int64_in', std_msgs.msg.Int64)
  i = 0
  r = rospy.Rate(10)
  m = std_msgs.msg.Int64(i)
  while not rospy.is_shutdown():
    pub.publish(m)
    i += 1
    m.data = i
    r.sleep()

if __name__ == '__main__':
  publisher()

#!/usr/bin/env python

# Extracts image from a ROSBAG.

import os

import rosbag
from rospy.rostime import Time
import cv2
from cv_bridge import CvBridge, CvBridgeError


save_dir = "./"

leftTime = Time(0,0)
rightTime = Time(0,0)
matchLeft = True
total = 0
savedCalibLeft = False
savedCalibRight = False

print "path is here"
print os.environ['LD_LIBRARY_PATH']

bag = rosbag.Bag('/home/pja/Desktop/ROSBAG/June16/2014-06-16-11-57-35.bag')
for topic, msg, t in bag.read_messages():
    print '----------------------------------------------------'
    print 't     '+str(t)
    print 'Topic '+topic
    #print 'Type  '+msg._type
    #print msg.header

    isLeft = "left" in topic

    if "image_rect" in topic:
        # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg)
        except CvBridgeError, e:
            print e

        save = False

        if isLeft:
            leftTime = msg.header.stamp
            leftImage = cv_image
            if not matchLeft:
                if leftTime == rightTime:
                    save = True
                else:
                    matchLeft = True

        else:
            rightTime = msg.header.stamp
            rightImage = cv_image
            if matchLeft:
                if leftTime == rightTime:
                    save = True
                else:
                    matchLeft = False

        if save:
            cv2.imwrite("images/left%05d.jpg"%total, leftImage , [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            cv2.imwrite("images/right%05d.jpg"%total, rightImage , [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            total = total + 1
    elif "camera_info":
        # save the calibration info
        if isLeft and not savedCalibLeft:
            leftP = msg.P[:]
            savedCalibLeft = True
        elif not isLeft and not savedCalibRight:
            rightP = msg.P[:]
            savedCalibRight = True

bag.close()
print "Done"

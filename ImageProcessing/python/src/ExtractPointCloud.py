#!/usr/bin/env python

# Extracts image from a ROSBAG.

import os

import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np


f = open('pointcloud.txt', 'w')

print "path is here"
print os.environ['LD_LIBRARY_PATH']

bag = rosbag.Bag('/home/pja/testArea/2014-07-09-15-28-18.bag')
for topic, msg, t in bag.read_messages():
    print '----------------------------------------------------'
    print 't     '+str(t)
    print 'Topic '+topic
    #print 'Type  '+msg._type
    #print msg.header


    if "lidar_points2" in topic:
        for x in pc2.read_points(msg):
            r = np.sqrt(np.dot(x[0:3],x[0:3]))

            # filter points which are outside of the max range of the sensor
            if r <= 30:
                f.write("{:.12f} {:.12f} {:.12f}\n".format(x[0],x[1],x[2]))

bag.close()
f.close()

print "Done"

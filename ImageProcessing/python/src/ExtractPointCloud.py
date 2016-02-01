#!/usr/bin/env python

# Extracts image from a ROSBAG.

import os

import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from os import listdir


dirName = "/home/pja/Desktop/lidar_2014-08-01"
fileList = [ f for f in listdir(dirName) if f.endswith('.bag') ]

for idx,fname in enumerate(fileList):
    print 'bag name = '+fname
    bag = rosbag.Bag(dirName+"/"+fname)
    f = open('cloud{:02d}.txt'.format(idx), 'w')

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

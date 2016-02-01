#!/usr/bin/env python

# Extracts image from a ROSBAG.

import os

import rosbag
import sensor_msgs.point_cloud2 as pc2
from os import listdir

dirName = "/home/pja/Desktop/lidar_2014-08-01"
fileList = [ f for f in listdir(dirName) if f.endswith('.bag') ]

for idx,fname in enumerate(fileList):
    print 'bag name = '+fname
    bag = rosbag.Bag(dirName+"/"+fname)
    f = open('cloud{:02d}_scans.txt'.format(idx), 'w')
    f.write('# Each scan is in the same reference frame.  One line per scan.\n')
    f.write('# NumPoints x1 y1 z1 ... xN yN zN\n')

    for topic, msg, t in bag.read_messages():
        print '----------------------------------------------------'
        print 't     '+str(t)
        print 'Topic '+topic
    #print 'Type  '+msg._type
    #print msg.header


        if "lidar_points2" in topic:
            list = [x for x in pc2.read_points(msg)]
            f.write("{0:d}".format(len(list)))
            for x in list:
                f.write(" {:.12f} {:.12f} {:.12f}".format(x[0],x[1],x[2]))
            f.write("\n")

    bag.close()
    f.close()

print "Done"

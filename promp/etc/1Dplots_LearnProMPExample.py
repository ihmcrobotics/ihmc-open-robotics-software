#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 30 16:22:57 2022

@author: luigi
"""

import numpy
import sys
import matplotlib.pyplot as plotter


#----------------LEARN PROMP------------------------------------------

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_demos=10
demo = []
for i in range(1,n_demos+1):
    demos_pathname = 'demo' + str(i) + '.csv'
    demo.append(numpy.genfromtxt(demos_pathname,delimiter=',', dtype = float))
mean = numpy.genfromtxt('mean.csv',delimiter=',', dtype = float)
stdDeviation = numpy.genfromtxt('stdDeviation.csv',delimiter=',', dtype = float)

mean_handX = numpy.array([row[0] for row in mean])
std_handX= numpy.array([row[0] for row in stdDeviation])
mean_handY = numpy.array([row[1] for row in mean])
std_handY = numpy.array([row[1] for row in stdDeviation])
mean_handZ = numpy.array([row[2] for row in mean])
std_handZ = numpy.array([row[2] for row in stdDeviation])

fig = plotter.figure()
colorX ='red'
colorY ='yellowgreen'
colorZ ='cornflowerblue'
democolorX='darkred'
democolorY='darkgreen'
democolorZ='midnightblue'

plotter.plot(mean_handX,colorX,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_handX.size, num=mean_handX.size), mean_handX - std_handX, mean_handX + std_handX, color=colorX, alpha=0.2)
plotter.plot(mean_handY,colorY,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_handY.size, num=mean_handY.size), mean_handY - std_handY, mean_handY + std_handY, color=colorY, alpha=0.2)
plotter.plot(mean_handZ,colorZ,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_handZ.size, num=mean_handZ.size), mean_handZ - std_handZ, mean_handZ + std_handZ, color=colorZ, alpha=0.2)
for i in range(0,n_demos):
    plotter.plot(demo[i][:,0],democolorX)
    plotter.plot(demo[i][:,1],democolorY)
    plotter.plot(demo[i][:,2],democolorZ)
plotter.xlabel('#samples')
plotter.ylabel('right hand [m]')
plotter.grid(True)



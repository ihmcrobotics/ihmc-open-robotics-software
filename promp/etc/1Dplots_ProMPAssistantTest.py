#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 7 14:22:57 2022

@author: luigi
"""

import numpy
import sys
import matplotlib.pyplot as plotter


#----------------LEARN PROMP------------------------------------------

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_demos=35
bodyParts = ['leftHand','rightHand']
for bodyPart in bodyParts:
    demo = []
    for i in range(1,n_demos+1):
        demos_pathname = bodyPart + 'demo' + str(i) + '.csv'
        demo.append(numpy.genfromtxt(demos_pathname,delimiter=',', dtype = float))
    mean = numpy.genfromtxt(bodyPart + 'mean.csv',delimiter=',', dtype = float)
    stdDeviation = numpy.genfromtxt(bodyPart + 'stdDeviation.csv',delimiter=',', dtype = float)
    
    mean_handQX = numpy.array([row[0] for row in mean])
    std_handQX = numpy.array([row[0] for row in stdDeviation])
    mean_handQY = numpy.array([row[1] for row in mean])
    std_handQY = numpy.array([row[1] for row in stdDeviation])
    mean_handQZ = numpy.array([row[2] for row in mean])
    std_handQZ = numpy.array([row[2] for row in stdDeviation])
    mean_handQS = numpy.array([row[3] for row in mean])
    std_handQS = numpy.array([row[3] for row in stdDeviation])
    mean_handX = numpy.array([row[4] for row in mean])
    std_handX= numpy.array([row[4] for row in stdDeviation])
    mean_handY = numpy.array([row[5] for row in mean])
    std_handY = numpy.array([row[5] for row in stdDeviation])
    mean_handZ = numpy.array([row[6] for row in mean])
    std_handZ = numpy.array([row[6] for row in stdDeviation])
    
    fig = plotter.figure()
    colorX ='red'
    colorY ='yellowgreen'
    colorZ ='cornflowerblue'
    colorS ='violet'
    democolorX='darkred'
    democolorY='darkgreen'
    democolorZ='midnightblue'
    democolorS='darkmagenta'
    
    for i in range(0,n_demos):
        plotter.plot(demo[i][:,4],democolorX)
        plotter.plot(demo[i][:,5],democolorY)
        plotter.plot(demo[i][:,6],democolorZ)
    plotter.plot(mean_handX,colorX,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handX.size, num=mean_handX.size), mean_handX - std_handX, mean_handX + std_handX, color=colorX, alpha=0.2)
    plotter.plot(mean_handY,colorY,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handY.size, num=mean_handY.size), mean_handY - std_handY, mean_handY + std_handY, color=colorY, alpha=0.2)
    plotter.plot(mean_handZ,colorZ,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handZ.size, num=mean_handZ.size), mean_handZ - std_handZ, mean_handZ + std_handZ, color=colorZ, alpha=0.2)
    plotter.xlabel('#samples')
    plotter.ylabel(str(bodyPart) + ' position [m]')
    plotter.grid(True)
    
    fig = plotter.figure()
    for i in range(0,n_demos):
        plotter.plot(demo[i][:,0],democolorX)
        plotter.plot(demo[i][:,1],democolorY)
        plotter.plot(demo[i][:,2],democolorZ)
        plotter.plot(demo[i][:,3],democolorS)
    plotter.plot(mean_handQX,colorX,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handQX.size, num=mean_handQX.size), mean_handQX - std_handQX, mean_handQX + std_handQX, color=colorX, alpha=0.2)
    plotter.plot(mean_handQY,colorY,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handQY.size, num=mean_handQY.size), mean_handQY - std_handQY, mean_handQY + std_handQY, color=colorY, alpha=0.2)
    plotter.plot(mean_handQZ,colorZ,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handQZ.size, num=mean_handQZ.size), mean_handQZ - std_handQZ, mean_handQZ + std_handQZ, color=colorZ, alpha=0.2)
    plotter.plot(mean_handQS,colorS,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handQS.size, num=mean_handQS.size), mean_handQS - std_handQS, mean_handQS + std_handQS, color=colorS, alpha=0.2)
    plotter.xlabel('#samples')
    plotter.ylabel(str(bodyPart) + ' quaternions [rad]')
    plotter.grid(True)



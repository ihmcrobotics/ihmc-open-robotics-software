#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 30 16:22:57 2022

@author: luigi
"""

import numpy as np
import sys
import matplotlib.pyplot as plotter


#----------------LEARN AND UPDATE PROMP------------------------------------------

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_demos=10
n_test=6
demo = []
test = []
for i in range(1,n_demos+1):
    demos_pathname = 'demo' + str(i) + '.csv'
    demo.append(np.genfromtxt(demos_pathname,delimiter=',', dtype = float))
for i in range(1,n_test+1):
    tests_pathname = 'test' + str(i) + '.csv'
    test.append(np.genfromtxt(tests_pathname,delimiter=',', dtype = float))
mean = np.genfromtxt('mean.csv',delimiter=',', dtype = float)
variance = np.genfromtxt('variance.csv',delimiter=',', dtype = float)

mean_handX = np.array([row[0] for row in mean])
std_handX= np.array([row[0] for row in variance])
mean_handY = np.array([row[1] for row in mean])
std_handY = np.array([row[1] for row in variance])
mean_handZ = np.array([row[2] for row in mean])
std_handZ = np.array([row[2] for row in variance])

fig = plotter.figure()
colorX ='red'
colorY ='yellowgreen'
colorZ ='cornflowerblue'
democolorX='darkred'
democolorY='darkgreen'
democolorZ='midnightblue'
testcolorX='coral'
testcolorY='olive'
testcolorZ='teal'

plotter.plot(mean_handX,colorX,linewidth=4.0)
plotter.fill_between(np.linspace(0, mean_handX.size, num=mean_handX.size), mean_handX - std_handX, mean_handX + std_handX, color=colorX, alpha=0.2)
plotter.plot(mean_handY,colorY,linewidth=4.0)
plotter.fill_between(np.linspace(0, mean_handY.size, num=mean_handY.size), mean_handY - std_handY, mean_handY + std_handY, color=colorY, alpha=0.2)
plotter.plot(mean_handZ,colorZ,linewidth=4.0)
plotter.fill_between(np.linspace(0, mean_handZ.size, num=mean_handZ.size), mean_handZ - std_handZ, mean_handZ + std_handZ, color=colorZ, alpha=0.2)
for i in range(0,n_demos):
    plotter.plot(demo[i][:,0],democolorX)
    plotter.plot(demo[i][:,1],democolorY)
    plotter.plot(demo[i][:,2],democolorZ)
for i in range(0,n_test):
    plotter.plot(test[i][:,0],testcolorX)
    plotter.plot(test[i][:,1],testcolorY)
    plotter.plot(test[i][:,2],testcolorZ)
plotter.xlabel('#samples')
plotter.ylabel('right hand position [m]')
plotter.grid(True)



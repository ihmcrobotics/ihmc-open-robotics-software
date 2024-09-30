#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 30 16:22:57 2022

@author: luigi
"""

import numpy
import sys
import matplotlib.pyplot as plotter


#----------------LEARN AND UPDATE PROMP------------------------------------------

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#------- Training and testing trajectories with learned ProMP ------
n_demos=10
n_test=6
demo = []
test = []
for i in range(1,n_demos+1):
    demos_pathname = 'demo' + str(i) + '.csv'
    demo.append(numpy.genfromtxt(demos_pathname,delimiter=',', dtype = float))
for i in range(1,n_test+1):
    tests_pathname = 'test' + str(i) + '.csv'
    test.append(numpy.genfromtxt(tests_pathname,delimiter=',', dtype = float))
mean = numpy.genfromtxt('mean.csv',delimiter=',', dtype = float)
stdDeviation = numpy.genfromtxt('stdDeviation.csv',delimiter=',', dtype = float)

mean_handX = numpy.array([row[0] for row in mean])
std_handX= numpy.array([row[0] for row in stdDeviation])
mean_handY = numpy.array([row[1] for row in mean])
std_handY = numpy.array([row[1] for row in stdDeviation])
mean_handZ = numpy.array([row[2] for row in mean])
std_handZ = numpy.array([row[2] for row in stdDeviation])

colorX ='red'
colorY ='yellowgreen'
colorZ ='cornflowerblue'
democolorX='darkred'
democolorY='darkgreen'
democolorZ='midnightblue'
testcolorX='coral'
testcolorY='olive'
testcolorZ='teal'

fig = plotter.figure()
fig.suptitle('Training and testing trajectories with learned ProMP', fontsize=16)
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
for i in range(0,n_test):
    plotter.plot(test[i][:,0],testcolorX)
    plotter.plot(test[i][:,1],testcolorY)
    plotter.plot(test[i][:,2],testcolorZ)
plotter.xlabel('#samples')
plotter.ylabel('right hand position [m]')
plotter.grid(True)
plotter.show()

#------- ProMP before and after modulation with observed (1/3) demo ------
meanModulated = numpy.genfromtxt('meanModulated.csv',delimiter=',', dtype = float)
stdDeviationModulated = numpy.genfromtxt('stdDeviationModulated.csv',delimiter=',', dtype = float)
# meanPreModulated = numpy.genfromtxt('meanPreModulated.csv',delimiter=',', dtype = float)
mean_modulated_handX = numpy.array([row[0] for row in meanModulated])
std_modulated_handX = numpy.array([row[0] for row in stdDeviationModulated])
mean_modulated_handY = numpy.array([row[1] for row in meanModulated])
std_modulated_handY = numpy.array([row[1] for row in stdDeviationModulated])
mean_modulated_handZ = numpy.array([row[2] for row in meanModulated])
std_modulated_handZ = numpy.array([row[2] for row in stdDeviationModulated])
# mean_premodulated_handX = numpy.array([row[0] for row in meanPreModulated])
# mean_premodulated_handY = numpy.array([row[1] for row in meanPreModulated])
# mean_premodulated_handZ = numpy.array([row[2] for row in meanPreModulated])

fig = plotter.figure()
fig.suptitle('ProMP before and after modulation with observed (1/3) demo', fontsize=16)
plotter.plot(mean_handX,colorX,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_handX.size, num=mean_handX.size), mean_handX - std_handX, mean_handX + std_handX, color=colorX, alpha=0.2)
plotter.plot(mean_handY,colorY,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_handY.size, num=mean_handY.size), mean_handY - std_handY, mean_handY + std_handY, color=colorY, alpha=0.2)
plotter.plot(mean_handZ,colorZ,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_handZ.size, num=mean_handZ.size), mean_handZ - std_handZ, mean_handZ + std_handZ, color=colorZ, alpha=0.2)

plotter.plot(test[0][:,0],testcolorX)
plotter.plot(test[0][:,1],testcolorY)
plotter.plot(test[0][:,2],testcolorZ)

plotter.plot(mean_modulated_handX,democolorX,linewidth=4.0)
plotter.plot(mean_modulated_handY,democolorY,linewidth=4.0)
plotter.plot(mean_modulated_handZ,democolorZ,linewidth=4.0)
# plotter.plot(mean_premodulated_handX,colorX,linewidth=4.0)
# plotter.plot(mean_premodulated_handY,colorY,linewidth=4.0)
# plotter.plot(mean_premodulated_handZ,colorZ,linewidth=4.0)
plotter.xlabel('#samples')
plotter.ylabel('right hand position [m]')
plotter.grid(True)
plotter.show()

#------- ProMP before and after conditioning update ------
meanConditioned = numpy.genfromtxt('meanConditioned.csv',delimiter=',', dtype = float)
stdDeviationConditioned = numpy.genfromtxt('stdDeviationConditioned.csv',delimiter=',', dtype = float)
mean_conditioned_handX = numpy.array([row[0] for row in meanConditioned])
std_conditioned_handX = numpy.array([row[0] for row in stdDeviationConditioned])
mean_conditioned_handY = numpy.array([row[1] for row in meanConditioned])
std_conditioned_handY = numpy.array([row[1] for row in stdDeviationConditioned])
mean_conditioned_handZ = numpy.array([row[2] for row in meanConditioned])
std_conditioned_handZ = numpy.array([row[2] for row in stdDeviationConditioned])

fig = plotter.figure()
fig.suptitle('ProMP before and after conditioning update', fontsize=16)
plotter.plot(mean_modulated_handX,colorX,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_modulated_handX.size, num=mean_modulated_handX.size), mean_modulated_handX - std_modulated_handX, mean_modulated_handX + std_modulated_handX, color=colorX, alpha=0.2)
plotter.plot(mean_modulated_handY,colorY,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_modulated_handY.size, num=mean_modulated_handY.size), mean_modulated_handY - std_modulated_handY, mean_modulated_handY + std_modulated_handY, color=colorY, alpha=0.2)
plotter.plot(mean_modulated_handZ,colorZ,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_modulated_handZ.size, num=mean_modulated_handZ.size), mean_modulated_handZ - std_modulated_handZ, mean_modulated_handZ + std_modulated_handZ, color=colorZ, alpha=0.2)

plotter.plot(mean_conditioned_handX,democolorX,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_conditioned_handX.size, num=mean_conditioned_handX.size), mean_conditioned_handX - std_conditioned_handX, mean_conditioned_handX + std_conditioned_handX, color=democolorX, alpha=0.2)
plotter.plot(mean_conditioned_handY,democolorY,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_conditioned_handY.size, num=mean_conditioned_handY.size), mean_conditioned_handY - std_conditioned_handY, mean_conditioned_handY + std_conditioned_handY, color=democolorY, alpha=0.2)
plotter.plot(mean_conditioned_handZ,democolorZ,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, mean_conditioned_handZ.size, num=mean_conditioned_handZ.size), mean_conditioned_handZ - std_conditioned_handZ, mean_conditioned_handZ + std_conditioned_handZ, color=democolorZ, alpha=0.2)

plotter.plot(test[0][:,0],testcolorX)
plotter.plot(test[0][:,1],testcolorY)
plotter.plot(test[0][:,2],testcolorZ)
plotter.xlabel('#samples')
plotter.ylabel('right hand position [m]')
plotter.grid(True)
plotter.show()





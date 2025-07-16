#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 7 14:22:57 2022

@author: luigi
"""

import numpy
import sys
import matplotlib.pyplot as plotter
import os


#----------------LEARN PROMP------------------------------------------

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_demos=20
colorX ='red'
colorY ='yellowgreen'
colorZ ='cornflowerblue'
colorS ='violet'
democolorX='darkred'
democolorY='darkgreen'
democolorZ='midnightblue'
democolorS='darkmagenta'
testcolorX='coral'
testcolorY='olive'
testcolorZ='teal'
testcolorS='palevioletred'

bodyPart= 'rightHand'

demo = []
for i in range(1,n_demos+1):
    demos_pathname = bodyPart + 'demo' + str(i) + '.csv'
    demo.append(numpy.genfromtxt(demos_pathname,delimiter=',', dtype = float))
mean = numpy.genfromtxt(bodyPart + 'mean.csv',delimiter=',', dtype = float)
stdDeviation = numpy.genfromtxt(bodyPart + 'stdDeviation.csv',delimiter=',', dtype = float)

meanQX = numpy.array([row[0] for row in mean])
stdQX = numpy.array([row[0] for row in stdDeviation])
meanQY = numpy.array([row[1] for row in mean])
stdQY = numpy.array([row[1] for row in stdDeviation])
meanQZ = numpy.array([row[2] for row in mean])
stdQZ = numpy.array([row[2] for row in stdDeviation])
meanQS = numpy.array([row[3] for row in mean])
stdQS = numpy.array([row[3] for row in stdDeviation])
meanX = numpy.array([row[4] for row in mean])
stdX= numpy.array([row[4] for row in stdDeviation])
meanY = numpy.array([row[5] for row in mean])
stdY = numpy.array([row[5] for row in stdDeviation])
meanZ = numpy.array([row[6] for row in mean])
stdZ = numpy.array([row[6] for row in stdDeviation])

fig = plotter.figure()
fig.suptitle('Training and testing trajectories with learned ProMP', fontsize=16)
for i in range(0,n_demos):
    plotter.plot(demo[i][:,4],democolorX)
    plotter.plot(demo[i][:,5],democolorY)
    plotter.plot(demo[i][:,6],democolorZ)
plotter.plot(meanX,colorX,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanX.size, num=meanX.size), meanX - stdX, meanX + stdX, color=colorX, alpha=0.2)
plotter.plot(meanY,colorY,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanY.size, num=meanY.size), meanY - stdY, meanY + stdY, color=colorY, alpha=0.2)
plotter.plot(meanZ,colorZ,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanZ.size, num=meanZ.size), meanZ - stdZ, meanZ + stdZ, color=colorZ, alpha=0.2)
plotter.xlabel('#samples')
plotter.ylabel(str(bodyPart) + ' position [m]')
plotter.grid(True)
plotter.savefig(bodyPart + 'PositionLearned.png')
plotter.show()

fig = plotter.figure()
fig.suptitle('Training and testing trajectories with learned ProMP', fontsize=16)
for i in range(0,n_demos):
    plotter.plot(demo[i][:,0],democolorX)
    plotter.plot(demo[i][:,1],democolorY)
    plotter.plot(demo[i][:,2],democolorZ)
    plotter.plot(demo[i][:,3],democolorS)
plotter.plot(meanQX,colorX,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanQX.size, num=meanQX.size), meanQX - stdQX, meanQX + stdQX, color=colorX, alpha=0.2)
plotter.plot(meanQY,colorY,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanQY.size, num=meanQY.size), meanQY - stdQY, meanQY + stdQY, color=colorY, alpha=0.2)
plotter.plot(meanQZ,colorZ,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanQZ.size, num=meanQZ.size), meanQZ - stdQZ, meanQZ + stdQZ, color=colorZ, alpha=0.2)
plotter.plot(meanQS,colorS,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanQS.size, num=meanQS.size), meanQS - stdQS, meanQS + stdQS, color=colorS, alpha=0.2)
plotter.xlabel('#samples')
plotter.ylabel(str(bodyPart) + ' quaternions [rad]')
plotter.grid(True)
plotter.savefig(bodyPart + 'OrientationLearned.png')
plotter.show()


#------- ProMP before and after modulation with observed demo ------
meanModulated = numpy.genfromtxt(bodyPart + 'meanModulated.csv',delimiter=',', dtype = float)
stdDeviationModulated = numpy.genfromtxt(bodyPart + 'stdDeviationModulated.csv',delimiter=',', dtype = float)
 
meanModulatedQX = numpy.array([row[0] for row in meanModulated])
stdModulatedQX = numpy.array([row[0] for row in stdDeviationModulated])
meanModulatedQY = numpy.array([row[1] for row in meanModulated])
stdModulatedQY = numpy.array([row[1] for row in stdDeviationModulated])
meanModulatedQZ = numpy.array([row[2] for row in meanModulated])
stdModulatedQZ = numpy.array([row[2] for row in stdDeviationModulated])
meanModulatedQS = numpy.array([row[3] for row in meanModulated])
stdModulatedQS = numpy.array([row[3] for row in stdDeviationModulated])
meanModulatedX = numpy.array([row[4] for row in meanModulated])
stdModulatedX = numpy.array([row[4] for row in stdDeviationModulated])
meanModulatedY = numpy.array([row[5] for row in meanModulated])
stdModulatedY = numpy.array([row[5] for row in stdDeviationModulated])
meanModulatedZ = numpy.array([row[6] for row in meanModulated])
stdModulatedZ = numpy.array([row[6] for row in stdDeviationModulated])

test = numpy.genfromtxt('test/DoorTest/test.csv',delimiter=',', dtype = float)

fig = plotter.figure()
fig.suptitle('ProMP before and after modulation with observed demo', fontsize=16)
plotter.plot(meanX,colorX,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanX.size, num=meanX.size), meanX - stdX, meanX + stdX, color=colorX, alpha=0.2)
plotter.plot(meanY,colorY,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanY.size, num=meanY.size), meanY - stdY, meanY + stdY, color=colorY, alpha=0.2)
plotter.plot(meanZ,colorZ,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanZ.size, num=meanZ.size), meanZ - stdZ, meanZ + stdZ, color=colorZ, alpha=0.2)

plotter.plot(test[:,11],testcolorX)
plotter.plot(test[:,12],testcolorY)
plotter.plot(test[:,13],testcolorZ)
    

plotter.plot(meanModulatedX,democolorX,linewidth=4.0)
plotter.plot(meanModulatedY,democolorY,linewidth=4.0)
plotter.plot(meanModulatedZ,democolorZ,linewidth=4.0)
plotter.xlabel('#samples')
plotter.ylabel(bodyPart + ' position [m]')
plotter.grid(True)
plotter.savefig(bodyPart + 'PositionModulated.png')
plotter.show()

fig = plotter.figure()
fig.suptitle('ProMP before and after modulation with observed demo', fontsize=16)
plotter.plot(meanQX,colorX,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanQX.size, num=meanQX.size), meanQX - stdQX, meanQX + stdQX, color=colorX, alpha=0.2)
plotter.plot(meanQY,colorY,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanQY.size, num=meanQY.size), meanQY - stdQY, meanQY + stdQY, color=colorY, alpha=0.2)
plotter.plot(meanQZ,colorZ,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanQZ.size, num=meanQZ.size), meanQZ - stdQZ, meanQZ + stdQZ, color=colorZ, alpha=0.2)
plotter.plot(meanQS,colorS,linewidth=4.0)
plotter.fill_between(numpy.linspace(0, meanQS.size, num=meanQS.size), meanQS - stdQS, meanQS + stdQS, color=colorS, alpha=0.2)

plotter.plot(test[:,7],testcolorX)
plotter.plot(test[:,8],testcolorY)
plotter.plot(test[:,9],testcolorZ)
plotter.plot(test[:,10],testcolorS)

plotter.plot(meanModulatedQX,democolorX,linewidth=4.0)
plotter.plot(meanModulatedQY,democolorY,linewidth=4.0)
plotter.plot(meanModulatedQZ,democolorZ,linewidth=4.0)
plotter.plot(meanModulatedQS,democolorS,linewidth=4.0)

plotter.xlabel('#samples')
plotter.ylabel(bodyPart + ' quaternions [rad]')
plotter.grid(True)
plotter.savefig(bodyPart + 'OrientationModulated.png')
plotter.show()



    
#------- Shared-control arbitared motion ------
motion = numpy.genfromtxt('generatedMotion.csv',delimiter=',', dtype = float)
partIndex = 1

alphas = []
for i in range(0,n_demos):
    demos_pathname = 'my_promp' + str(i) + '.csv'
    alphas.append(numpy.genfromtxt(demos_pathname,delimiter=',', dtype = float))
    
for i in range(0,n_demos):
    fig = plotter.figure()
    fig.suptitle('Shared-control motion' + str(i), fontsize=16)
    plotter.plot(alphas[i][:100,4],democolorX)
    plotter.plot(alphas[i][:100,5],democolorY)
    plotter.plot(alphas[i][:100,6],democolorZ)
    plotter.plot(test[:100,11],testcolorX,linewidth=1.0)
    plotter.plot(test[:100,12],testcolorY,linewidth=1.0)
    plotter.plot(test[:100,13],testcolorZ,linewidth=1.0)
      
    plotter.xlabel('#samples')
    plotter.ylabel(str(bodyPart) + ' position [m]')
    plotter.grid(True)
    plotter.savefig(bodyPart + 'Position.png')
    plotter.show()
    
for i in range(0,n_demos):
    fig = plotter.figure()
    fig.suptitle('Shared-control motion'+ str(i), fontsize=16)
      
    plotter.plot(alphas[i][:100,0],democolorX)
    plotter.plot(alphas[i][:100,1],democolorY)
    plotter.plot(alphas[i][:100,2],democolorZ)
    plotter.plot(alphas[i][:100,3],democolorZ)
    plotter.plot(test[:100,7],testcolorX,linewidth=1.0)
    plotter.plot(test[:100,8],testcolorY,linewidth=1.0)
    plotter.plot(test[:100,9],testcolorZ,linewidth=1.0)
    plotter.plot(test[:100,10],testcolorS,linewidth=1.0)
    
    plotter.xlabel('#samples')
    plotter.ylabel(str(bodyPart) + ' quaternions [rad]')
    plotter.grid(True)
    plotter.savefig(bodyPart + 'Orientation.png')
    plotter.show()


    
    
    
    




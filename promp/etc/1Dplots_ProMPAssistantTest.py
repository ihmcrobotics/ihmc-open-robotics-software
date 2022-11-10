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
n_demos=35
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
    fig.suptitle('Training and testing trajectories with learned ProMP', fontsize=16)
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
    fig.suptitle('Training and testing trajectories with learned ProMP', fontsize=16)
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
    
    
    #------- ProMP before and after modulation with observed demo ------
    meanModulated = numpy.genfromtxt(bodyPart + 'meanModulated.csv',delimiter=',', dtype = float)
    stdDeviationModulated = numpy.genfromtxt(bodyPart + 'stdDeviationModulated.csv',delimiter=',', dtype = float)
 
    mean_modulated_handQX = numpy.array([row[0] for row in meanModulated])
    std_modulated_handQX = numpy.array([row[0] for row in stdDeviationModulated])
    mean_modulated_handQY = numpy.array([row[1] for row in meanModulated])
    std_modulated_handQY = numpy.array([row[1] for row in stdDeviationModulated])
    mean_modulated_handQZ = numpy.array([row[2] for row in meanModulated])
    std_modulated_handQZ = numpy.array([row[2] for row in stdDeviationModulated])
    mean_modulated_handQS = numpy.array([row[3] for row in meanModulated])
    std_modulated_handQS = numpy.array([row[3] for row in stdDeviationModulated])
    mean_modulated_handX = numpy.array([row[4] for row in meanModulated])
    std_modulated_handX = numpy.array([row[4] for row in stdDeviationModulated])
    mean_modulated_handY = numpy.array([row[5] for row in meanModulated])
    std_modulated_handY = numpy.array([row[5] for row in stdDeviationModulated])
    mean_modulated_handZ = numpy.array([row[6] for row in meanModulated])
    std_modulated_handZ = numpy.array([row[6] for row in stdDeviationModulated])
    
    test = numpy.genfromtxt('test/PushDoorTest/1.csv',delimiter=',', dtype = float)

    fig = plotter.figure()
    fig.suptitle('ProMP before and after modulation with observed demo', fontsize=16)
    plotter.plot(mean_handX,colorX,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handX.size, num=mean_handX.size), mean_handX - std_handX, mean_handX + std_handX, color=colorX, alpha=0.2)
    plotter.plot(mean_handY,colorY,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handY.size, num=mean_handY.size), mean_handY - std_handY, mean_handY + std_handY, color=colorY, alpha=0.2)
    plotter.plot(mean_handZ,colorZ,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handZ.size, num=mean_handZ.size), mean_handZ - std_handZ, mean_handZ + std_handZ, color=colorZ, alpha=0.2)

    if bodyPart == bodyParts[0]:
        plotter.plot(test[:,4],testcolorX)
        plotter.plot(test[:,5],testcolorY)
        plotter.plot(test[:,6],testcolorZ)
    elif bodyPart == bodyParts[1]:
        plotter.plot(test[:,11],testcolorX)
        plotter.plot(test[:,12],testcolorY)
        plotter.plot(test[:,13],testcolorZ)
        

    plotter.plot(mean_modulated_handX,democolorX,linewidth=4.0)
    plotter.plot(mean_modulated_handY,democolorY,linewidth=4.0)
    plotter.plot(mean_modulated_handZ,democolorZ,linewidth=4.0)
    plotter.xlabel('#samples')
    plotter.ylabel(bodyPart + ' position [m]')
    plotter.grid(True)
    plotter.show()
    
    fig = plotter.figure()
    fig.suptitle('ProMP before and after modulation with observed demo', fontsize=16)
    plotter.plot(mean_handQX,colorX,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handQX.size, num=mean_handQX.size), mean_handQX - std_handQX, mean_handQX + std_handQX, color=colorX, alpha=0.2)
    plotter.plot(mean_handQY,colorY,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handQY.size, num=mean_handQY.size), mean_handQY - std_handQY, mean_handQY + std_handQY, color=colorY, alpha=0.2)
    plotter.plot(mean_handQZ,colorZ,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handQZ.size, num=mean_handQZ.size), mean_handQZ - std_handQZ, mean_handQZ + std_handQZ, color=colorZ, alpha=0.2)
    plotter.plot(mean_handQS,colorS,linewidth=4.0)
    plotter.fill_between(numpy.linspace(0, mean_handQS.size, num=mean_handQS.size), mean_handQS - std_handQS, mean_handQS + std_handQS, color=colorS, alpha=0.2)
    
    if bodyPart == bodyParts[0]:
        plotter.plot(test[:,0],testcolorX)
        plotter.plot(test[:,1],testcolorY)
        plotter.plot(test[:,2],testcolorZ)
        plotter.plot(test[:,3],testcolorS)
    elif bodyPart == bodyParts[1]:
        plotter.plot(test[:,7],testcolorX)
        plotter.plot(test[:,8],testcolorY)
        plotter.plot(test[:,9],testcolorZ)
        plotter.plot(test[:,10],testcolorS)

    plotter.plot(mean_modulated_handQX,democolorX,linewidth=4.0)
    plotter.plot(mean_modulated_handQY,democolorY,linewidth=4.0)
    plotter.plot(mean_modulated_handQZ,democolorZ,linewidth=4.0)
    plotter.plot(mean_modulated_handQS,democolorS,linewidth=4.0)
    
    plotter.xlabel('#samples')
    plotter.ylabel(bodyPart + ' quaternions [rad]')
    plotter.grid(True)
    plotter.show()
    
    
    #------- ProMP before and after conditioning update ------
    exists = os.path.isfile(bodyPart + 'meanConditioned.csv')
    if exists:
        meanConditioned = numpy.genfromtxt(bodyPart + 'meanConditioned.csv',delimiter=',', dtype = float)
        stdDeviationConditioned = numpy.genfromtxt(bodyPart + 'stdDeviationConditioned.csv',delimiter=',', dtype = float)
        
        mean_conditioned_handQX = numpy.array([row[0] for row in meanConditioned])
        std_conditioned_handQX = numpy.array([row[0] for row in stdDeviationConditioned])
        mean_conditioned_handQY = numpy.array([row[1] for row in meanConditioned])
        std_conditioned_handQY = numpy.array([row[1] for row in stdDeviationConditioned])
        mean_conditioned_handQZ = numpy.array([row[2] for row in meanConditioned])
        std_conditioned_handQZ = numpy.array([row[2] for row in stdDeviationConditioned])
        mean_conditioned_handQS = numpy.array([row[3] for row in meanConditioned])
        std_conditioned_handQS = numpy.array([row[3] for row in stdDeviationConditioned])
        mean_conditioned_handX = numpy.array([row[4] for row in meanConditioned])
        std_conditioned_handX = numpy.array([row[4] for row in stdDeviationConditioned])
        mean_conditioned_handY = numpy.array([row[5] for row in meanConditioned])
        std_conditioned_handY = numpy.array([row[5] for row in stdDeviationConditioned])
        mean_conditioned_handZ = numpy.array([row[6] for row in meanConditioned])
        std_conditioned_handZ = numpy.array([row[6] for row in stdDeviationConditioned])
    
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
    
        if bodyPart == bodyParts[0]:
            plotter.plot(test[:,4],testcolorX)
            plotter.plot(test[:,5],testcolorY)
            plotter.plot(test[:,6],testcolorZ)
        elif bodyPart == bodyParts[1]:
            plotter.plot(test[:,11],testcolorX)
            plotter.plot(test[:,12],testcolorY)
            plotter.plot(test[:,13],testcolorZ)
        
        plotter.xlabel('#samples')
        plotter.ylabel(bodyPart + ' position [m]')
        plotter.grid(True)
        plotter.show()
        
        fig = plotter.figure()
        fig.suptitle('ProMP before and after conditioning update', fontsize=16)
        plotter.plot(mean_handQX,colorX,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, mean_handQX.size, num=mean_handQX.size), mean_handQX - std_handQX, mean_handQX + std_handQX, color=colorX, alpha=0.2)
        plotter.plot(mean_handQY,colorY,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, mean_handQY.size, num=mean_handQY.size), mean_handQY - std_handQY, mean_handQY + std_handQY, color=colorY, alpha=0.2)
        plotter.plot(mean_handQZ,colorZ,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, mean_handQZ.size, num=mean_handQZ.size), mean_handQZ - std_handQZ, mean_handQZ + std_handQZ, color=colorZ, alpha=0.2)
        plotter.plot(mean_handQS,colorS,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, mean_handQS.size, num=mean_handQS.size), mean_handQS - std_handQS, mean_handQS + std_handQS, color=colorS, alpha=0.2)
        
        plotter.plot(mean_conditioned_handQX,democolorX,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, mean_conditioned_handQX.size, num=mean_conditioned_handQX.size), mean_conditioned_handQX - std_conditioned_handQX, mean_conditioned_handQX + std_conditioned_handQX, color=democolorX, alpha=0.2)
        plotter.plot(mean_conditioned_handQY,democolorY,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, mean_conditioned_handQY.size, num=mean_conditioned_handQY.size), mean_conditioned_handQY - std_conditioned_handQY, mean_conditioned_handQY + std_conditioned_handQY, color=democolorY, alpha=0.2)
        plotter.plot(mean_conditioned_handQZ,democolorZ,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, mean_conditioned_handQZ.size, num=mean_conditioned_handQZ.size), mean_conditioned_handQZ - std_conditioned_handQZ, mean_conditioned_handQZ + std_conditioned_handQZ, color=democolorZ, alpha=0.2)
        plotter.plot(mean_conditioned_handQS,democolorS,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, mean_conditioned_handQS.size, num=mean_conditioned_handQS.size), mean_conditioned_handQS - std_conditioned_handQS, mean_conditioned_handQS + std_conditioned_handQS, color=democolorS, alpha=0.2)
        
        if bodyPart == bodyParts[0]:
            plotter.plot(test[:,0],testcolorX)
            plotter.plot(test[:,1],testcolorY)
            plotter.plot(test[:,2],testcolorZ)
            plotter.plot(test[:,3],testcolorS)
        elif bodyPart == bodyParts[1]:
            plotter.plot(test[:,7],testcolorX)
            plotter.plot(test[:,8],testcolorY)
            plotter.plot(test[:,9],testcolorZ)
            plotter.plot(test[:,10],testcolorS)
        
        plotter.xlabel('#samples')
        plotter.ylabel(bodyPart + ' quaternions [rad]')
        plotter.grid(True)
        plotter.show()
    
    
    
    




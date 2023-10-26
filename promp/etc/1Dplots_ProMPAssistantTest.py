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

n_demos=57
bodyParts = ['leftHand','rightHand','chest']
# bodyParts = ['leftHand','rightHand']
for bodyPart in bodyParts:
    if 'right' not in bodyPart:
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
        if 'Hand' in bodyPart:
            meanX = numpy.array([row[4] for row in mean])
            stdX= numpy.array([row[4] for row in stdDeviation])
            meanY = numpy.array([row[5] for row in mean])
            stdY = numpy.array([row[5] for row in stdDeviation])
            meanZ = numpy.array([row[6] for row in mean])
            stdZ = numpy.array([row[6] for row in stdDeviation])
        
        if 'Hand' in bodyPart:
            fig = plotter.figure()
            fig.suptitle('Training trajectories with learned ProMP', fontsize=16)
            for i in range(0,n_demos):
                if i==0:
                    plotter.plot(demo[i][:,4],democolorX,label='demo')
                else:
                    plotter.plot(demo[i][:,4],democolorX)
                plotter.plot(demo[i][:,5],democolorY)
                plotter.plot(demo[i][:,6],democolorZ)
            plotter.plot(meanX,colorX,linewidth=4.0, label='ProMP mean')
            plotter.fill_between(numpy.linspace(0, meanX.size, num=meanX.size), meanX - stdX, meanX + stdX, color=colorX, alpha=0.2, label='ProMP std')
            plotter.plot(meanY,colorY,linewidth=4.0)
            plotter.fill_between(numpy.linspace(0, meanY.size, num=meanY.size), meanY - stdY, meanY + stdY, color=colorY, alpha=0.2)
            plotter.plot(meanZ,colorZ,linewidth=4.0)
            plotter.fill_between(numpy.linspace(0, meanZ.size, num=meanZ.size), meanZ - stdZ, meanZ + stdZ, color=colorZ, alpha=0.2)
            plotter.xlabel('#samples')
            plotter.ylabel(str(bodyPart) + ' position [m]')
            plotter.grid(True)
            legend = fig.legend(loc='lower right', shadow=True, fontsize='x-small')
            plotter.savefig(bodyPart + 'PositionLearned.png')
            plotter.savefig(bodyPart + 'PositionLearned.svg')
            plotter.show()
        
        fig = plotter.figure()
        fig.suptitle('Training trajectories with learned ProMP', fontsize=16)
        for i in range(0,n_demos):
            if i==0:
                plotter.plot(demo[i][:,0],democolorX,label='demo')
            else:
                plotter.plot(demo[i][:,0],democolorX)
            plotter.plot(demo[i][:,1],democolorY)
            plotter.plot(demo[i][:,2],democolorZ)
            plotter.plot(demo[i][:,3],democolorS)
        plotter.plot(meanQX,colorX,linewidth=4.0, label='ProMP mean')
        plotter.fill_between(numpy.linspace(0, meanQX.size, num=meanQX.size), meanQX - stdQX, meanQX + stdQX, color=colorX, alpha=0.2, label='ProMP std')
        plotter.plot(meanQY,colorY,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, meanQY.size, num=meanQY.size), meanQY - stdQY, meanQY + stdQY, color=colorY, alpha=0.2)
        plotter.plot(meanQZ,colorZ,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, meanQZ.size, num=meanQZ.size), meanQZ - stdQZ, meanQZ + stdQZ, color=colorZ, alpha=0.2)
        plotter.plot(meanQS,colorS,linewidth=4.0)
        plotter.fill_between(numpy.linspace(0, meanQS.size, num=meanQS.size), meanQS - stdQS, meanQS + stdQS, color=colorS, alpha=0.2)
        plotter.xlabel('#samples')
        plotter.ylabel(str(bodyPart) + ' quaternions [rad]')
        plotter.grid(True)
        legend = fig.legend(loc='lower right', shadow=True, fontsize='x-small')
        plotter.savefig(bodyPart + 'OrientationLearned.png')
        plotter.savefig(bodyPart + 'OrientationLearned.svg')
        plotter.show()
        
        
        # #------- ProMP before and after modulation with observed demo ------
        # meanModulated = numpy.genfromtxt(bodyPart + 'meanModulated.csv',delimiter=',', dtype = float)
        # stdDeviationModulated = numpy.genfromtxt(bodyPart + 'stdDeviationModulated.csv',delimiter=',', dtype = float)
     
        # meanModulatedQX = numpy.array([row[0] for row in meanModulated])
        # stdModulatedQX = numpy.array([row[0] for row in stdDeviationModulated])
        # meanModulatedQY = numpy.array([row[1] for row in meanModulated])
        # stdModulatedQY = numpy.array([row[1] for row in stdDeviationModulated])
        # meanModulatedQZ = numpy.array([row[2] for row in meanModulated])
        # stdModulatedQZ = numpy.array([row[2] for row in stdDeviationModulated])
        # meanModulatedQS = numpy.array([row[3] for row in meanModulated])
        # stdModulatedQS = numpy.array([row[3] for row in stdDeviationModulated])
        # meanModulatedX = numpy.array([row[4] for row in meanModulated])
        # stdModulatedX = numpy.array([row[4] for row in stdDeviationModulated])
        # meanModulatedY = numpy.array([row[5] for row in meanModulated])
        # stdModulatedY = numpy.array([row[5] for row in stdDeviationModulated])
        # meanModulatedZ = numpy.array([row[6] for row in meanModulated])
        # stdModulatedZ = numpy.array([row[6] for row in stdDeviationModulated])
        
        test = numpy.genfromtxt('test/test.csv',delimiter=',', dtype = float)
    
        # fig = plotter.figure()
        # fig.suptitle('ProMP before and after modulation with observation', fontsize=16)
        # plotter.plot(meanX,colorX,linewidth=4.0)
        # plotter.fill_between(numpy.linspace(0, meanX.size, num=meanX.size), meanX - stdX, meanX + stdX, color=colorX, alpha=0.2)
        # plotter.plot(meanY,colorY,linewidth=4.0)
        # plotter.fill_between(numpy.linspace(0, meanY.size, num=meanY.size), meanY - stdY, meanY + stdY, color=colorY, alpha=0.2)
        # plotter.plot(meanZ,colorZ,linewidth=4.0,label='ProMP mean before')
        # plotter.fill_between(numpy.linspace(0, meanZ.size, num=meanZ.size), meanZ - stdZ, meanZ + stdZ, color=colorZ, alpha=0.2)
    
        # if bodyPart == bodyParts[0]:
        #     plotter.plot(test[:,4],testcolorX)
        #     plotter.plot(test[:,5],testcolorY)
        #     plotter.plot(test[:,6],testcolorZ,label='actual trajectory')
        # elif bodyPart == bodyParts[1]:
        #     plotter.plot(test[:,11],testcolorX)
        #     plotter.plot(test[:,12],testcolorY)
        #     plotter.plot(test[:,13],testcolorZ,label='actual trajectory')
            
    
        # plotter.plot(meanModulatedX,democolorX,linewidth=4.0)
        # plotter.plot(meanModulatedY,democolorY,linewidth=4.0)
        # plotter.plot(meanModulatedZ,democolorZ,linewidth=4.0,label='ProMP mean after')
        # plotter.xlabel('#samples')
        # plotter.ylabel(bodyPart + ' position [m]')
        # plotter.grid(True)
        # legend = fig.legend(loc='lower right', shadow=True, fontsize='x-small')
        # plotter.savefig(bodyPart + 'PositionModulated.png')
        # plotter.show()
        
        # fig = plotter.figure()
        # fig.suptitle('ProMP before and after modulation with observation', fontsize=16)
        # plotter.plot(meanQX,colorX,linewidth=4.0)
        # plotter.fill_between(numpy.linspace(0, meanQX.size, num=meanQX.size), meanQX - stdQX, meanQX + stdQX, color=colorX, alpha=0.2)
        # plotter.plot(meanQY,colorY,linewidth=4.0)
        # plotter.fill_between(numpy.linspace(0, meanQY.size, num=meanQY.size), meanQY - stdQY, meanQY + stdQY, color=colorY, alpha=0.2)
        # plotter.plot(meanQZ,colorZ,linewidth=4.0,label='ProMP mean before')
        # plotter.fill_between(numpy.linspace(0, meanQZ.size, num=meanQZ.size), meanQZ - stdQZ, meanQZ + stdQZ, color=colorZ, alpha=0.2)
        # plotter.plot(meanQS,colorS,linewidth=4.0)
        # plotter.fill_between(numpy.linspace(0, meanQS.size, num=meanQS.size), meanQS - stdQS, meanQS + stdQS, color=colorS, alpha=0.2)
        
        # if bodyPart == bodyParts[0]:
        #     plotter.plot(test[:,0],testcolorX)
        #     plotter.plot(test[:,1],testcolorY)
        #     plotter.plot(test[:,2],testcolorZ,label='actual trajectory')
        #     plotter.plot(test[:,3],testcolorS)
        # elif bodyPart == bodyParts[1]:
        #     plotter.plot(test[:,7],testcolorX)
        #     plotter.plot(test[:,8],testcolorY)
        #     plotter.plot(test[:,9],testcolorZ,label='actual trajectory')
        #     plotter.plot(test[:,10],testcolorS)
    
        # plotter.plot(meanModulatedQX,democolorX,linewidth=4.0)
        # plotter.plot(meanModulatedQY,democolorY,linewidth=4.0)
        # plotter.plot(meanModulatedQZ,democolorZ,linewidth=4.0,label='ProMP mean after')
        # plotter.plot(meanModulatedQS,democolorS,linewidth=4.0)
        
        # plotter.xlabel('#samples')
        # plotter.ylabel(bodyPart + ' quaternions [rad]')
        # plotter.grid(True)
        # legend = fig.legend(loc='lower right', shadow=True, fontsize='x-small')
        # plotter.savefig(bodyPart + 'OrientationModulated.png')
        # plotter.show()
        
        
        #------- ProMP before and after conditioning update ------
        exists = os.path.isfile(bodyPart + 'meanConditioned.csv')
        if exists:
            meanConditioned = numpy.genfromtxt(bodyPart + 'meanConditioned.csv',delimiter=',', dtype = float)
            stdDeviationConditioned = numpy.genfromtxt(bodyPart + 'stdDeviationConditioned.csv',delimiter=',', dtype = float)
            
            meanConditionedQX = numpy.array([row[0] for row in meanConditioned])
            stdConditionedQX = numpy.array([row[0] for row in stdDeviationConditioned])
            meanConditionedQY = numpy.array([row[1] for row in meanConditioned])
            stdConditionedQY = numpy.array([row[1] for row in stdDeviationConditioned])
            meanConditionedQZ = numpy.array([row[2] for row in meanConditioned])
            stdConditionedQZ = numpy.array([row[2] for row in stdDeviationConditioned])
            meanConditionedQS = numpy.array([row[3] for row in meanConditioned])
            stdConditionedQS = numpy.array([row[3] for row in stdDeviationConditioned])
            if 'Hand' in bodyPart:
                meanConditionedX = numpy.array([row[4] for row in meanConditioned])
                stdConditionedX = numpy.array([row[4] for row in stdDeviationConditioned])
                meanConditionedY = numpy.array([row[5] for row in meanConditioned])
                stdConditionedY = numpy.array([row[5] for row in stdDeviationConditioned])
                meanConditionedZ = numpy.array([row[6] for row in meanConditioned])
                stdConditionedZ = numpy.array([row[6] for row in stdDeviationConditioned])
        
            if 'Hand' in bodyPart:
                fig = plotter.figure()
                fig.suptitle('ProMP before and after conditioning update', fontsize=16)
                plotter.plot(meanX,colorX,linewidth=4.0)
                plotter.fill_between(numpy.linspace(0, meanX.size, num=meanX.size), meanX - stdX, meanX + stdX, color=colorX, alpha=0.2)
                plotter.plot(meanY,colorY,linewidth=4.0)
                plotter.fill_between(numpy.linspace(0, meanY.size, num=meanY.size), meanY - stdY, meanY + stdY, color=colorY, alpha=0.2)
                plotter.plot(meanZ,colorZ,linewidth=4.0,label='ProMP mean before')
                plotter.fill_between(numpy.linspace(0, meanZ.size, num=meanZ.size), meanZ - stdZ, meanZ + stdZ, color=colorZ, alpha=0.2)
            
                plotter.plot(meanConditionedX,democolorX,linewidth=4.0)
                plotter.fill_between(numpy.linspace(0, meanConditionedX.size, num=meanConditionedX.size), meanConditionedX - stdConditionedX, meanConditionedX + stdConditionedX, color=democolorX, alpha=0.2)
                plotter.plot(meanConditionedY,democolorY,linewidth=4.0)
                plotter.fill_between(numpy.linspace(0, meanConditionedY.size, num=meanConditionedY.size), meanConditionedY - stdConditionedY, meanConditionedY + stdConditionedY, color=democolorY, alpha=0.2)
                plotter.plot(meanConditionedZ,democolorZ,linewidth=4.0,label='ProMP mean after')
                plotter.fill_between(numpy.linspace(0, meanConditionedZ.size, num=meanConditionedZ.size), meanConditionedZ - stdConditionedZ, meanConditionedZ + stdConditionedZ, color=democolorZ, alpha=0.2)
            
                if bodyPart == bodyParts[0]:
                    plotter.plot(test[:,4],testcolorX)
                    plotter.plot(test[:,5],testcolorY)
                    plotter.plot(test[:,6],testcolorZ,label='actual trajectory')
                elif bodyPart == bodyParts[1]:
                    plotter.plot(test[:,11],testcolorX)
                    plotter.plot(test[:,12],testcolorY)
                    plotter.plot(test[:,13],testcolorZ,label='actual trajectory')
            
                
                plotter.xlabel('#samples')
                plotter.ylabel(bodyPart + ' position [m]')
                plotter.grid(True)
                legend = fig.legend(loc='lower right', shadow=True, fontsize='x-small')
                plotter.savefig(bodyPart + 'PositionConditioned.png')
                plotter.savefig(bodyPart + 'PositionConditioned.svg')
                plotter.show()
            
            fig = plotter.figure()
            fig.suptitle('ProMP before and after conditioning update', fontsize=16)
            plotter.plot(meanQX,colorX,linewidth=4.0)
            plotter.fill_between(numpy.linspace(0, meanQX.size, num=meanQX.size), meanQX - stdQX, meanQX + stdQX, color=colorX, alpha=0.2)
            plotter.plot(meanQY,colorY,linewidth=4.0)
            plotter.fill_between(numpy.linspace(0, meanQY.size, num=meanQY.size), meanQY - stdQY, meanQY + stdQY, color=colorY, alpha=0.2)
            plotter.plot(meanQZ,colorZ,linewidth=4.0,label='ProMP mean before')
            plotter.fill_between(numpy.linspace(0, meanQZ.size, num=meanQZ.size), meanQZ - stdQZ, meanQZ + stdQZ, color=colorZ, alpha=0.2)
            plotter.plot(meanQS,colorS,linewidth=4.0)
            plotter.fill_between(numpy.linspace(0, meanQS.size, num=meanQS.size), meanQS - stdQS, meanQS + stdQS, color=colorS, alpha=0.2)
            
            plotter.plot(meanConditionedQX,democolorX,linewidth=4.0)
            plotter.fill_between(numpy.linspace(0, meanConditionedQX.size, num=meanConditionedQX.size), meanConditionedQX - stdConditionedQX, meanConditionedQX + stdConditionedQX, color=democolorX, alpha=0.2)
            plotter.plot(meanConditionedQY,democolorY,linewidth=4.0)
            plotter.fill_between(numpy.linspace(0, meanConditionedQY.size, num=meanConditionedQY.size), meanConditionedQY - stdConditionedQY, meanConditionedQY + stdConditionedQY, color=democolorY, alpha=0.2)
            plotter.plot(meanConditionedQZ,democolorZ,linewidth=4.0,label='ProMP mean after')
            plotter.fill_between(numpy.linspace(0, meanConditionedQZ.size, num=meanConditionedQZ.size), meanConditionedQZ - stdConditionedQZ, meanConditionedQZ + stdConditionedQZ, color=democolorZ, alpha=0.2)
            plotter.plot(meanConditionedQS,democolorS,linewidth=4.0)
            plotter.fill_between(numpy.linspace(0, meanConditionedQS.size, num=meanConditionedQS.size), meanConditionedQS - stdConditionedQS, meanConditionedQS + stdConditionedQS, color=democolorS, alpha=0.2)
            
            if bodyPart == bodyParts[0]:
                plotter.plot(test[:,0],testcolorX)
                plotter.plot(test[:,1],testcolorY)
                plotter.plot(test[:,2],testcolorZ,label='actual trajectory')
                plotter.plot(test[:,3],testcolorS)
            elif bodyPart == bodyParts[1]:
                plotter.plot(test[:,7],testcolorX)
                plotter.plot(test[:,8],testcolorY)
                plotter.plot(test[:,9],testcolorZ,label='actual trajectory')
                plotter.plot(test[:,10],testcolorS)
            elif bodyPart == bodyParts[2]:
                plotter.plot(test[:,14],testcolorX)
                plotter.plot(test[:,15],testcolorY)
                plotter.plot(test[:,16],testcolorZ,label='actual trajectory')
                plotter.plot(test[:,17],testcolorS)
            elif bodyPart == bodyParts[3]:
                plotter.plot(test[:,21],testcolorX)
                plotter.plot(test[:,22],testcolorY)
                plotter.plot(test[: 23],testcolorZ,label='actual trajectory')
                plotter.plot(test[:,24],testcolorS)
            elif bodyPart == bodyParts[4]:
                plotter.plot(test[:,28],testcolorX)
                plotter.plot(test[:,29],testcolorY)
                plotter.plot(test[:,30],testcolorZ,label='actual trajectory')
                plotter.plot(test[:,31],testcolorS)
            
            plotter.xlabel('#samples')
            plotter.ylabel(bodyPart + ' quaternions [rad]')
            plotter.grid(True)
            legend = fig.legend(loc='lower right', shadow=True, fontsize='x-small')
            plotter.savefig(bodyPart + 'OrientationConditioned.png')
            plotter.savefig(bodyPart + 'OrientationConditioned.svg')
            plotter.show()
            
        #------- Shared-control arbitared motion ------
        # motion = numpy.genfromtxt('generatedMotion.csv',delimiter=',', dtype = float)
        # partIndex = bodyParts.index(bodyPart)
        
        # qX = numpy.array([row[0+7*partIndex] for row in motion])
        # qY = numpy.array([row[1+7*partIndex] for row in motion])
        # qZ = numpy.array([row[2+7*partIndex] for row in motion])
        # qS = numpy.array([row[3+7*partIndex] for row in motion])
        # if 'Hand' in bodyPart:
        #     x = numpy.array([row[4+7*partIndex] for row in motion])
        #     y = numpy.array([row[5+7*partIndex] for row in motion])
        #     z = numpy.array([row[6+7*partIndex] for row in motion])
        
        #     fig = plotter.figure()
        #     fig.suptitle('Shared-control motion', fontsize=16)
        #     plotter.plot(x,colorX,linewidth=1.0)
        #     plotter.plot(y,colorY,linewidth=1.0)
        #     plotter.plot(z,colorZ,linewidth=1.0,label='final executed trajectory')
        #     plotter.plot(meanConditionedX,democolorX,linewidth=1.0)
        #     plotter.plot(meanConditionedY,democolorY,linewidth=1.0)
        #     plotter.plot(meanConditionedZ,democolorZ,linewidth=1.0,label='ProMP mean conditioned')
        #     if bodyPart == bodyParts[0]:
        #         plotter.plot(test[:,4],testcolorX,linewidth=1.0)
        #         plotter.plot(test[:,5],testcolorY,linewidth=1.0)
        #         plotter.plot(test[:,6],testcolorZ,linewidth=1.0,label='user trajectory')
        #     elif bodyPart == bodyParts[1]:
        #         plotter.plot(test[:,11],testcolorX,linewidth=1.0)
        #         plotter.plot(test[:,12],testcolorY,linewidth=1.0)
        #         plotter.plot(test[:,13],testcolorZ,linewidth=1.0,label='user trajectory')
          
        #     plotter.xlabel('#samples')
        #     plotter.ylabel(str(bodyPart) + ' position [m]')
        #     plotter.grid(True)
        #     legend = fig.legend(loc='lower right', shadow=True, fontsize='x-small')
        #     plotter.savefig(bodyPart + 'Position.png')
        #     plotter.show()
        
        # fig = plotter.figure()
        # fig.suptitle('Shared-control motion', fontsize=16)
      
        # plotter.plot(qX,colorX,linewidth=1.0)
        # plotter.plot(qY,colorY,linewidth=1.0)
        # plotter.plot(qZ,colorZ,linewidth=1.0,label='final executed trajectory')
        # plotter.plot(qS,colorS,linewidth=1.0)
        # plotter.plot(meanConditionedQX,democolorX,linewidth=1.0)
        # plotter.plot(meanConditionedQY,democolorY,linewidth=1.0)
        # plotter.plot(meanConditionedQZ,democolorZ,linewidth=1.0,label='ProMP mean conditioned')
        # plotter.plot(meanConditionedQS,democolorS,linewidth=1.0)
        # if bodyPart == bodyParts[0]:
        #     plotter.plot(test[:,0],testcolorX,linewidth=1.0)
        #     plotter.plot(test[:,1],testcolorY,linewidth=1.0)
        #     plotter.plot(test[:,2],testcolorZ,linewidth=1.0,label='user trajectory')
        #     plotter.plot(test[:,3],testcolorS,linewidth=1.0)
        # elif bodyPart == bodyParts[1]:
        #     plotter.plot(test[:,7],testcolorX,linewidth=1.0)
        #     plotter.plot(test[:,8],testcolorY,linewidth=1.0)
        #     plotter.plot(test[:,9],testcolorZ,linewidth=1.0,label='user trajectory')
        #     plotter.plot(test[:,10],testcolorS,linewidth=1.0)
        
        # plotter.xlabel('#samples')
        # plotter.ylabel(str(bodyPart) + ' quaternions [rad]')
        # plotter.grid(True)
        # legend = fig.legend(loc='lower right', shadow=True, fontsize='x-small')
        # plotter.savefig(bodyPart + 'Orientation.png')
        # plotter.show()

    
    
    
    



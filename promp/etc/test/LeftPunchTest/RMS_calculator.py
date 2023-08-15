# -*- coding: utf-8 -*-
"""
Created on Thu Aug  3 17:28:41 2023

@author: lpenco
"""
import numpy
import sys
import matplotlib.pyplot as plotter
import os
import math

bodyParts = ['leftHand','leftForeArm', 'chest']
for bodyPart in bodyParts:
    test = []
    testQX = []
    testQY = []
    testQZ = []
    testQS = []
    testX = []
    testY = []
    testZ = []
    
    meanConditionedQX = []
    meanConditionedQY = []
    meanConditionedQZ = []
    meanConditionedQS = []
    meanConditionedX = []
    meanConditionedY = []
    meanConditionedZ = []
    
    rmsQX = []
    rmsQY = []
    rmsQZ = []
    rmsQS = []
    
    rmsX = []
    rmsY = []
    rmsZ = []
    
    overallRmsRad = []
    overallRmsCm = []
    
    # create test matrix
    for i in range(0,10):
        tests_pathname = str(i+1) + '.csv'
        test.append(numpy.genfromtxt(tests_pathname,delimiter=',', dtype = float))
        if bodyPart == bodyParts[0]:
            shift = 0
        elif bodyPart == bodyParts[1]:
            shift = 14
        elif bodyPart == bodyParts[2]:
            shift = 28
        testQX.append(test[i][:, shift + 0]) 
        testQY.append(test[i][:, shift + 1]) 
        testQZ.append(test[i][:, shift + 2])  
        testQS.append(test[i][:, shift + 3])
       
        if 'Hand' in bodyPart:
            testX.append(test[i][:, shift + 4]) 
            testY.append(test[i][:, shift + 5]) 
            testZ.append(test[i][:, shift + 6]) 
      
    # create conditioned (adapted) promp matrix
    for i in range(0,10):
        exists = os.path.isfile('conditioned/' + str(i+1) + '/' + bodyPart + 'meanConditioned.csv')
        if exists:
            meanConditioned = numpy.genfromtxt('conditioned/' + str(i+1) + '/' + bodyPart + 'meanConditioned.csv',delimiter=',', dtype = float)      
            
            meanConditionedQX.append(numpy.array([row[0] for row in meanConditioned]))
            meanConditionedQY.append(numpy.array([row[1] for row in meanConditioned]))
            meanConditionedQZ.append(numpy.array([row[2] for row in meanConditioned]))
            meanConditionedQS.append(numpy.array([row[3] for row in meanConditioned]))
            if 'Hand' in bodyPart:
                meanConditionedX.append(numpy.array([row[4] for row in meanConditioned]))
                meanConditionedY.append(numpy.array([row[5] for row in meanConditioned]))
                meanConditionedZ.append(numpy.array([row[6] for row in meanConditioned]))
               
    #compute RMS for each promp and test trajectory
    for i in range(0,10):
        rmsQX.append(numpy.sqrt(numpy.mean((testQX[i] - meanConditionedQX[i]) ** 2)))
        rmsQY.append(numpy.sqrt(numpy.mean((testQY[i] - meanConditionedQY[i]) ** 2)))
        rmsQZ.append(numpy.sqrt(numpy.mean((testQZ[i] - meanConditionedQZ[i]) ** 2)))
        rmsQS.append(numpy.sqrt(numpy.mean((testQS[i] - meanConditionedQS[i]) ** 2)))
        if 'Hand' in bodyPart:
            rmsX.append(numpy.sqrt(numpy.mean((testX[i] - meanConditionedX[i]) ** 2)))
            rmsY.append(numpy.sqrt(numpy.mean((testY[i] - meanConditionedY[i]) ** 2)))
            rmsZ.append(numpy.sqrt(numpy.mean((testZ[i] - meanConditionedZ[i]) ** 2)))
            
        overallRmsRad.append(math.sqrt((rmsQS[i]*rmsQS[i] + rmsQX[i]*rmsQX[i] + rmsQY[i]*rmsQY[i] + rmsQZ[i]*rmsQZ[i]) / 4))
        if 'Hand' in bodyPart:
            overallRmsCm.append(math.sqrt((rmsX[i]*rmsX[i] + rmsY[i]*rmsY[i] + rmsZ[i]*rmsZ[i]) / 3) / 0.01)
            
    if 'Hand' in bodyPart:
         # Calculate the mean and standard deviation of the components of overallRmsRad
         meanOverallRmsCm = numpy.mean(overallRmsCm)
         stdOverallRmsCm = numpy.std(overallRmsCm)
         # Print the results
         print(f"Body Part: {bodyPart}")
         print(f"Mean of Overall RMS (cm): {meanOverallRmsCm:.2f}")
         print(f"Standard Deviation of Overall RMS (cm): {stdOverallRmsCm:.2f}")
    # Calculate the mean and standard deviation of the components of overallRmsRad
    meanOverallRmsRad = numpy.mean(overallRmsRad)
    stdOverallRmsRad = numpy.std(overallRmsRad)
    # Print the results
    print(f"Body Part: {bodyPart}")
    print(f"Mean of Overall RMS (rad): {meanOverallRmsRad:.2f}")
    print(f"Standard Deviation of Overall RMS (rad): {stdOverallRmsRad:.2f}")
        
          
            
                
    
        
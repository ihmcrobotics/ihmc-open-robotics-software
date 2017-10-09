#!/usr/bin/env python

from Numeric import *


cstParams = 'startupDropCount=0 readingDataPointsFilterCount=2   readingStepDataPointsFilterCount=0 keyframeDataPointsFilterCount=2  keyframeDataPointsFilter/1/name=SamplingSurfaceNormalDataPointsFilter  matcher/name=KDTreeMatcher transformationCheckerCount=2 transformationCheckers/0/name=ErrorTransformationChecker transformationCheckers/0/minRotErrorDelta=0.001 transformationCheckers/0/minTransErrorDelta=0.01 transformationCheckers/0/tail=4 transformationCheckers/1/name=CounterTransformationChecker transformationCheckers/1/maxIterationCount=40 featureOutlierFilterCount=2 featureOutlierFilters/0/name=MaxDistOutlierFilter featureOutlierFilters/0/maxDist=0.2 featureOutlierFilters/1/name=MedianDistOutlierFilter featureOutlierFilters/1/factor=4 inspector/name=Inspector errorMinimizer/name=PointToPlaneErrorMinimizer ratioToSwitchKeyframe=0.7 matcher/epsilon=0 keyframeDataPointsFilter/1/k=17 readingDataPointsFilter/1/name=RandomSamplingDataPointsFilter readingDataPointsFilter/1/prob=0.5'


for test in range(15):
  for thres in arange(0.5, 5.5, 0.25):
    localParams = ' keyframeDataPointsFilter/0/name=ClampOnAxisThresholdDataPointsFilter keyframeDataPointsFilter/0/dim=2'
    localParams = localParams + ' keyframeDataPointsFilter/0/threshold=' + str(thres)
    localParams = localParams + ' readingDataPointsFilter/0/name=ClampOnAxisThresholdDataPointsFilter readingDataPointsFilter/0/dim=2'
    localParams = localParams + ' readingDataPointsFilter/0/threshold=' +str(thres) 
    print(cstParams + ' ' + localParams)

  for ratio in arange(0.05, 1.0, 0.05):
    localParams = ' keyframeDataPointsFilter/0/name=ClampOnAxisRatioDataPointsFilter keyframeDataPointsFilter/0/dim=2'
    localParams = localParams + ' keyframeDataPointsFilter/0/ratio=' + str(ratio)
    localParams = localParams + ' readingDataPointsFilter/0/name=ClampOnAxisRatioDataPointsFilter readingDataPointsFilter/0/dim=2'
    localParams = localParams + ' readingDataPointsFilter/0/ratio=' +str(ratio) 
    print(cstParams + ' ' + localParams)

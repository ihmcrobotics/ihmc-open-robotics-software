#!/bin/sh
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/LeeGoswamiForceOptimizerNative.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.LeeGoswamiForceOptimizerNative
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/LeeGoswamiCoPAndNormalTorqueOptimizerNative.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.LeeGoswamiCoPAndNormalTorqueOptimizerNative


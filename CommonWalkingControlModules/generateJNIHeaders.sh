#!/bin/sh
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/LeeGoswamiForceOptimizerNative.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.LeeGoswamiForceOptimizerNative
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/LeeGoswamiCoPAndNormalTorqueOptimizerNative.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.LeeGoswamiCoPAndNormalTorqueOptimizerNative
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/ContactPointWrenchOptimizerNative.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.ContactPointWrenchOptimizerNative
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/MomentumOptimizerNative.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNative
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/CVXWithCylinderNative.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXWithCylinderNative
javah -classpath ../IHMCUtilities/classes:./classes -o csrc/CylinderAndPlaneContactForceOptimizerNative.h us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CylinderAndPlaneContactForceOptimizerNative

package us.ihmc.openAlexander.parameters.controller;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class OpenAlexanderControllerParameters
{
   public static YoRegistry build()
   {
      // Root registry
      YoRegistry parameters = new YoRegistry("parameters");

      // ## Begin DRCControllerThread
      YoRegistry drcControllerThread = new YoRegistry("DRCControllerThread");
      parameters.addChild(drcControllerThread);

      YoRegistry drcMomentumBasedController = new YoRegistry("DRCMomentumBasedController");
      drcControllerThread.addChild(drcMomentumBasedController);

      YoRegistry humanoidHighLevelControllerManager = new YoRegistry("HumanoidHighLevelControllerManager");
      drcMomentumBasedController.addChild(humanoidHighLevelControllerManager);

      // ### WalkingControllerState
      YoRegistry walkingControllerState = new YoRegistry("WalkingControllerState");
      humanoidHighLevelControllerManager.addChild(walkingControllerState);

      YoRegistry walkingHighLevelHumanoidController = new YoRegistry("WalkingHighLevelHumanoidController");
      walkingControllerState.addChild(walkingHighLevelHumanoidController);

      YoRegistry touchdownErrorCompensator = new YoRegistry("TouchdownErrorCompensator");
      walkingHighLevelHumanoidController.addChild(touchdownErrorCompensator);
      YoDouble spatialVelocityThresholdForSupportConfidence = new YoDouble("spatialVelocityThresholdForSupportConfidence", touchdownErrorCompensator);
      spatialVelocityThresholdForSupportConfidence.set(0.4);
      YoDouble touchdownErrorCorrectionScale = new YoDouble("touchdownErrorCorrectionScale", touchdownErrorCompensator);
      touchdownErrorCorrectionScale.set(0.8);

      // ### LinearMomentumRateControlModule
      YoRegistry linearMomentumRateControlModule = new YoRegistry("LinearMomentumRateControlModule");
      walkingControllerState.addChild(linearMomentumRateControlModule);

      YoRegistry icpOptimizationController = new YoRegistry("ICPOptimizationController");
      linearMomentumRateControlModule.addChild(icpOptimizationController);
      YoDouble icpOptControllerThresholdForStuck = new YoDouble("controllerThresholdForStuck", icpOptimizationController);
      icpOptControllerThresholdForStuck.set(0.01);

      YoRegistry icpController = new YoRegistry("ICPController");
      linearMomentumRateControlModule.addChild(icpController);
      YoDouble icpControllerThresholdForStuck = new YoDouble("controllerThresholdForStuck", icpController);
      icpControllerThresholdForStuck.set(0.01);
      YoDouble captureKi = new YoDouble("captureKi", icpController);
      captureKi.set(1.0);
      YoDouble captureIntegralLeakRatio = new YoDouble("captureIntegralLeakRatio", icpController);
      captureIntegralLeakRatio.set(0.97);

      YoBoolean allowMomentumRecoveryWeight = new YoBoolean("allowMomentumRecoveryWeight", linearMomentumRateControlModule);
      allowMomentumRecoveryWeight.set(true);
      YoDouble linearMomentumRateWeightX = new YoDouble("LinearMomentumRateWeightX", linearMomentumRateControlModule);
      linearMomentumRateWeightX.set(0.075);
      YoDouble linearMomentumRateWeightY = new YoDouble("LinearMomentumRateWeightY", linearMomentumRateControlModule);
      linearMomentumRateWeightY.set(0.075);
      YoDouble linearMomentumRateWeightZ = new YoDouble("LinearMomentumRateWeightZ", linearMomentumRateControlModule);
      linearMomentumRateWeightZ.set(0.02);
      YoDouble recoveryLinearMomentumRateWeightX = new YoDouble("RecoveryLinearMomentumRateWeightX", linearMomentumRateControlModule);
      recoveryLinearMomentumRateWeightX.set(0.5);
      YoDouble recoveryLinearMomentumRateWeightY = new YoDouble("RecoveryLinearMomentumRateWeightY", linearMomentumRateControlModule);
      recoveryLinearMomentumRateWeightY.set(0.5);
      YoDouble recoveryLinearMomentumRateWeightZ = new YoDouble("RecoveryLinearMomentumRateWeightZ", linearMomentumRateControlModule);
      recoveryLinearMomentumRateWeightZ.set(0.01);
      YoDouble angularMomentumRateWeightX = new YoDouble("AngularMomentumRateWeightX", linearMomentumRateControlModule);
      angularMomentumRateWeightX.set(0.0);
      YoDouble angularMomentumRateWeightY = new YoDouble("AngularMomentumRateWeightY", linearMomentumRateControlModule);
      angularMomentumRateWeightY.set(0.0);
      YoDouble angularMomentumRateWeightZ = new YoDouble("AngularMomentumRateWeightZ", linearMomentumRateControlModule);
      angularMomentumRateWeightZ.set(0.0);

      // ### HighLevelHumanoidControllerFactory
      YoRegistry highLevelHumanoidControllerFactory = new YoRegistry("HighLevelHumanoidControllerFactory");
      humanoidHighLevelControllerManager.addChild(highLevelHumanoidControllerFactory);

      YoRegistry momentumOptimizationSettings = new YoRegistry("MomentumOptimizationSettings");
      highLevelHumanoidControllerFactory.addChild(momentumOptimizationSettings);
      YoDouble spineYawJointspaceWeight = new YoDouble("SpineYawJointspaceWeight", momentumOptimizationSettings);
      spineYawJointspaceWeight.set(15.0);
      YoDouble spinePitchJointspaceWeight = new YoDouble("SpinePitchJointspaceWeight", momentumOptimizationSettings);
      spinePitchJointspaceWeight.set(45.0);
      // ... (The rest of the variables are converted in the same pattern)
      YoDouble spineRollJointspaceWeight = new YoDouble("SpineRollJointspaceWeight", momentumOptimizationSettings);
      spineRollJointspaceWeight.set(45.0);
      YoDouble armsJointspaceWeight = new YoDouble("ArmsJointspaceWeight", momentumOptimizationSettings);
      armsJointspaceWeight.set(1.0);
      YoDouble legsJointspaceWeight = new YoDouble("LegsJointspaceWeight", momentumOptimizationSettings);
      legsJointspaceWeight.set(1.0);
      YoDouble neckJointspaceWeight = new YoDouble("NeckJointspaceWeight", momentumOptimizationSettings);
      neckJointspaceWeight.set(1.0);
      YoDouble spineUserModeWeight = new YoDouble("SpineUserModeWeight", momentumOptimizationSettings);
      spineUserModeWeight.set(200.0);
      YoDouble armsUserModeWeight = new YoDouble("ArmsUserModeWeight", momentumOptimizationSettings);
      armsUserModeWeight.set(50.0);
      YoDouble legsUserModeWeight = new YoDouble("LegsUserModeWeight", momentumOptimizationSettings);
      legsUserModeWeight.set(50.0);
      YoDouble neckUserModeWeight = new YoDouble("NeckUserModeWeight", momentumOptimizationSettings);
      neckUserModeWeight.set(1.0);
      YoDouble chestAngularWeightX = new YoDouble("ChestAngularWeightX", momentumOptimizationSettings);
      chestAngularWeightX.set(15.0);
      YoDouble chestAngularWeightY = new YoDouble("ChestAngularWeightY", momentumOptimizationSettings);
      chestAngularWeightY.set(10.0);
      YoDouble chestAngularWeightZ = new YoDouble("ChestAngularWeightZ", momentumOptimizationSettings);
      chestAngularWeightZ.set(5.0);
      YoDouble headAngularWeightX = new YoDouble("HeadAngularWeightX", momentumOptimizationSettings);
      headAngularWeightX.set(1.0);
      YoDouble headAngularWeightY = new YoDouble("HeadAngularWeightY", momentumOptimizationSettings);
      headAngularWeightY.set(1.0);
      YoDouble headAngularWeightZ = new YoDouble("HeadAngularWeightZ", momentumOptimizationSettings);
      headAngularWeightZ.set(1.0);
      YoDouble pelvisAngularWeightX = new YoDouble("PelvisAngularWeightX", momentumOptimizationSettings);
      pelvisAngularWeightX.set(5.0);
      YoDouble pelvisAngularWeightY = new YoDouble("PelvisAngularWeightY", momentumOptimizationSettings);
      pelvisAngularWeightY.set(5.0);
      YoDouble pelvisAngularWeightZ = new YoDouble("PelvisAngularWeightZ", momentumOptimizationSettings);
      pelvisAngularWeightZ.set(5.0);
      YoDouble handAngularWeightX = new YoDouble("HandAngularWeightX", momentumOptimizationSettings);
      handAngularWeightX.set(25.0);
      YoDouble handAngularWeightY = new YoDouble("HandAngularWeightY", momentumOptimizationSettings);
      handAngularWeightY.set(25.0);
      YoDouble handAngularWeightZ = new YoDouble("HandAngularWeightZ", momentumOptimizationSettings);
      handAngularWeightZ.set(25.0);
      YoDouble footAngularWeightX = new YoDouble("FootAngularWeightX", momentumOptimizationSettings);
      footAngularWeightX.set(0.5);
      YoDouble footAngularWeightY = new YoDouble("FootAngularWeightY", momentumOptimizationSettings);
      footAngularWeightY.set(0.5);
      YoDouble footAngularWeightZ = new YoDouble("FootAngularWeightZ", momentumOptimizationSettings);
      footAngularWeightZ.set(0.5);
      YoDouble pelvisLinearWeightX = new YoDouble("PelvisLinearWeightX", momentumOptimizationSettings);
      pelvisLinearWeightX.set(5.0);
      YoDouble pelvisLinearWeightY = new YoDouble("PelvisLinearWeightY", momentumOptimizationSettings);
      pelvisLinearWeightY.set(5.0);
      YoDouble pelvisLinearWeightZ = new YoDouble("PelvisLinearWeightZ", momentumOptimizationSettings);
      pelvisLinearWeightZ.set(50.0);
      YoDouble handLinearWeightX = new YoDouble("HandLinearWeightX", momentumOptimizationSettings);
      handLinearWeightX.set(25.0);
      YoDouble handLinearWeightY = new YoDouble("HandLinearWeightY", momentumOptimizationSettings);
      handLinearWeightY.set(25.0);
      YoDouble handLinearWeightZ = new YoDouble("HandLinearWeightZ", momentumOptimizationSettings);
      handLinearWeightZ.set(25.0);
      YoDouble footLinearWeightX = new YoDouble("FootLinearWeightX", momentumOptimizationSettings);
      footLinearWeightX.set(30.0);
      YoDouble footLinearWeightY = new YoDouble("FootLinearWeightY", momentumOptimizationSettings);
      footLinearWeightY.set(30.0);
      YoDouble footLinearWeightZ = new YoDouble("FootLinearWeightZ", momentumOptimizationSettings);
      footLinearWeightZ.set(30.0);
      YoDouble loadedFootAngularWeightX = new YoDouble("LoadedFootAngularWeightX", momentumOptimizationSettings);
      loadedFootAngularWeightX.set(5.0);
      YoDouble loadedFootAngularWeightY = new YoDouble("LoadedFootAngularWeightY", momentumOptimizationSettings);
      loadedFootAngularWeightY.set(5.0);
      YoDouble loadedFootAngularWeightZ = new YoDouble("LoadedFootAngularWeightZ", momentumOptimizationSettings);
      loadedFootAngularWeightZ.set(5.0);
      YoDouble loadedFootLinearWeightX = new YoDouble("LoadedFootLinearWeightX", momentumOptimizationSettings);
      loadedFootLinearWeightX.set(50.0);
      YoDouble loadedFootLinearWeightY = new YoDouble("LoadedFootLinearWeightY", momentumOptimizationSettings);
      loadedFootLinearWeightY.set(50.0);
      YoDouble loadedFootLinearWeightZ = new YoDouble("LoadedFootLinearWeightZ", momentumOptimizationSettings);
      loadedFootLinearWeightZ.set(50.0);

      // ... The rest of the file follows the same updated pattern ...

      // ## Begin DRCEstimatorThread
      YoRegistry drcEstimatorThread = new YoRegistry("DRCEstimatorThread");
      parameters.addChild(drcEstimatorThread);

      YoRegistry atlasHeadPoseEstimator = new YoRegistry("AtlasHeadPoseEstimator");
      drcEstimatorThread.addChild(atlasHeadPoseEstimator);
      YoDouble angularVelocityVariance = new YoDouble("AngularVelocityVariance", atlasHeadPoseEstimator);
      angularVelocityVariance.set(0.003);
      YoDouble linearAccelerationVariance = new YoDouble("LinearAccelerationVariance", atlasHeadPoseEstimator);
      linearAccelerationVariance.set(3.0E-4);
      YoDouble magneticFieldVariance = new YoDouble("MagneticFieldVariance", atlasHeadPoseEstimator);
      magneticFieldVariance.set(1000.0);
      YoDouble positionVariance = new YoDouble("PositionVariance", atlasHeadPoseEstimator);
      positionVariance.set(1000.0);
      YoDouble headAngularAccelerationVariance = new YoDouble("HeadAngularAccelerationVariance", atlasHeadPoseEstimator);
      headAngularAccelerationVariance.set(1000.0);
      YoDouble headLinearAccelerationVariance = new YoDouble("HeadLinearAccelerationVariance", atlasHeadPoseEstimator);
      headLinearAccelerationVariance.set(1000.0);
      YoDouble linearAccelerationBiasVariance = new YoDouble("LinearAccelerationBiasVariance", atlasHeadPoseEstimator);
      linearAccelerationBiasVariance.set(0.0);

      return parameters;
   }
}
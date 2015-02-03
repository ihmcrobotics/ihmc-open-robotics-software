package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.utilities.Pair;


public interface StateEstimatorParameters extends SensorProcessingConfiguration
{
   public abstract boolean isRunningOnRealRobot();
   
   public abstract double getEstimatorDT();
   
   public abstract boolean trustCoPAsNonSlippingContactPoint();

   public abstract boolean useControllerDesiredCenterOfPressure();

   public boolean useIMUsForSpineJointVelocityEstimation();
   public double getAlphaIMUsForSpineJointVelocityEstimation();
   public abstract Pair<String, String> getIMUsForSpineJointVelocityEstimation();
   
   // Parameters related to the kinematics based state estimator
   public abstract double getKinematicsPelvisPositionFilterFreqInHertz();
   public abstract double getKinematicsPelvisLinearVelocityFilterFreqInHertz();

   public abstract double getCoPFilterFreqInHertz();
   
   public abstract boolean useAccelerometerForEstimation();
   
   public abstract boolean estimateGravity();
   
   public abstract double getGravityFilterFreqInHertz();

   public abstract double getPelvisPositionFusingFrequency();
   public abstract double getPelvisLinearVelocityFusingFrequency();
   public abstract double getPelvisVelocityBacklashSlopTime();
   
   public abstract double getDelayTimeForTrustingFoot();
   
   public abstract double getForceInPercentOfWeightThresholdToTrustFoot();
   
   public abstract boolean estimateIMUDrift();

   public abstract boolean compensateIMUDrift();
   
   public abstract double getIMUDriftFilterFreqInHertz();
   
   public abstract double getFootVelocityUsedForImuDriftFilterFreqInHertz();
   
   public abstract double getFootVelocityThresholdToEnableIMUDriftCompensation();

   public abstract boolean useTwistForPelvisLinearStateEstimation();

   public abstract double getPelvisLinearVelocityAlphaNewTwist();
   
   public abstract boolean createFusedIMUSensor();

   public abstract double getContactThresholdForce();

   public abstract double getFootSwitchCoPThresholdFraction();

   public abstract double getContactThresholdHeight();

   public abstract FootSwitchType getFootSwitchType();
}

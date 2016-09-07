package us.ihmc.sensorProcessing.stateEstimation;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class StateEstimatorParameters implements SensorProcessingConfiguration
{
   public abstract boolean isRunningOnRealRobot();

   @Override
   public abstract double getEstimatorDT();

   public abstract boolean trustCoPAsNonSlippingContactPoint();

   public boolean useControllerDesiredCenterOfPressure()
   {
      return false;
   }

   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      return false;
   }

   /** @deprecated Need to switch to frequency */
   @Deprecated
   public double getAlphaIMUsForSpineJointVelocityEstimation()
   {
      return Double.NaN;
   }

   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      return null;
   }

   public double getIMUJointVelocityEstimationBacklashSlopTime()
   {
      return 0.0;
   }

   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return false;
   }

   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }

   public abstract boolean requestFootForceSensorCalibrationAtStart();

   public boolean requestFrozenModeAtStart()
   {
      return false;
   }

   public abstract SideDependentList<String> getFootForceSensorNames();

   // Parameters related to the kinematics based state estimator
   public abstract double getKinematicsPelvisPositionFilterFreqInHertz();

   public abstract double getCoPFilterFreqInHertz();

   public abstract boolean enableIMUBiasCompensation();

   public abstract boolean enableIMUYawDriftCompensation();

   public abstract double getIMUBiasFilterFreqInHertz();

   public abstract double getIMUYawDriftFilterFreqInHertz();

   public abstract double getIMUBiasVelocityThreshold();

   public abstract boolean useAccelerometerForEstimation();

   public abstract boolean cancelGravityFromAccelerationMeasurement();

   public abstract double getPelvisPositionFusingFrequency();

   public abstract double getPelvisLinearVelocityFusingFrequency();

   public abstract double getCenterOfMassVelocityFusingFrequency();

   public abstract double getDelayTimeForTrustingFoot();

   public abstract double getForceInPercentOfWeightThresholdToTrustFoot();

   public abstract double getPelvisLinearVelocityAlphaNewTwist();

   public boolean createFusedIMUSensor()
   {
      return false;
   }

   public abstract double getContactThresholdForce();

   public abstract double getFootSwitchCoPThresholdFraction();

   public abstract double getContactThresholdHeight();

   public abstract FootSwitchType getFootSwitchType();

   public abstract boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact();

   public abstract boolean useGroundReactionForcesToComputeCenterOfMassVelocity();

   public boolean correctTrustedFeetPositions()
   {
      return false;
   }
}

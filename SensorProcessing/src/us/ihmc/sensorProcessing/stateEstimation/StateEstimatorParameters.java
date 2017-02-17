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

   /**
    * Used to enable/disabled the IMUYawDriftEstimator
    */
   public boolean enableIMUYawDriftCompensation()
   {
      return false;
   }

   /**
    * When the IMU yaw drift cannot be estimated, the estimated rate of the yaw drift can be used
    * to partially compensate for the drift.
    * Works well when the yaw drift is linear.
    * When false, the yaw drift will only be compensated whn the robot is standing.
    */
   public boolean integrateEstimatedIMUYawDriftRate()
   {
      return false;
   }

   /**
    * The estimation of the yaw drift uses the position of the feet that are loaded and static.
    * This is the delay from the moment a foot becomes loaded and the moment it is used in the estimation.
    */
   public double getIMUYawDriftEstimatorDelayBeforeTrustingFoot()
   {
      return 0.5;
   }

   /**
    * The estimation of the yaw drift uses the position of the feet that are loaded and static.
    * This the maximum value for the linear velocity magnitude for each foot.
    * The estimation will start when all feet are loaded and static.
    */
   public double getIMUYawDriftFootLinearVelocityThreshold()
   {
      return 0.04;
   }

   /**
    * Filter break frequency for the estimation of the yaw drift.
    */
   public double getIMUYawDriftFilterFreqInHertz()
   {
      return 0.8;
   }

   /**
    * Filter break frequency for the estimation of the yaw rate drift.
    */
   public double getIMUYawDriftRateFilterFreqInHertz()
   {
      return 1.5e-3;
   }

   public abstract boolean enableIMUBiasCompensation();

   public abstract double getIMUBiasFilterFreqInHertz();

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
